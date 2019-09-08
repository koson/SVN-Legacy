/*
 * Name: Thruster_Kill_Board (TKB)
 * Author: Marquez Jones
 * Date Started: 3/22/19
 * Desc: Firmware for thruster/kill board
 *
 * NOTE: WHEN READING THIS CODE THE MEANING OF KILL
 *       CHANGED WHILE WRITING CODE. PLEASE REFER
 *       TO CORRESPONDING COMMENTS FOR CLARITY
 *
 * Terminology:
 * TKB - Thruster Kill Board (functions written for this project found in
 *                            Thurster_Kill_Board.h and .c)
 *
 * Task List:
 * Forward thruster commands from Mobo to Thrusters
 * Kill if motherboard heart beat lost
 * Kill if kill command received via CAN
 * Kill if Hard/Soft kill hall effects are removed
 * Report go status to motherboard
 *
 *
 * SCHEMATIC CONVENTIONS FROM FRANK
 * F - FRONT
 * B - BACK
 * R - RIGHT
 * L - LEFT
 * H - HORIZONTAL
 * V - VERTICAL
 *
 * THRUSTER PIN MAPPING:
 *  ALL PWM MODULE 0
 *  FH   Gen1   L-PWM3(B5)   R-PWM2(B4)
 *  FV   Gen2   L-PWM4(E4)   R-PWM5(E5)
 *  BH   Gen0   L-PWM0(B6)   R-PWM1(B7)
 *  BV   Gen3   L-PWM7(C5)   R-PWM6(C4)
 *
 *  ID MAPPING:
 *  FHL - 0
 *  FHR - 1
 *  FVL - 2
 *  FVR - 3
 *  BHL - 4
 *  BHR - 5
 *  BVL - 6
 *  BVR - 7
 *
 * NOTE: NAMING CONVENTIONS DO NOT FOLLOW
 *       THE SCHEMATIC.
 *
 *       THE PIN NAMES IN THE SOFTWARE
 *       ARE AKIN TO TEAM TERMINOLOGY
 *       AS OPPOSED TO THE HARDWARE TERMS
 *
 *       THIS IS THE SCHEMATIC TO FIRMWARE PIN MAP:
 *       FIRMARE         SCHEMATIC             PIN
 *       HALL_SOFTKILL -  HALL_SOFTKILL        :PB1
 *       HALL_GO       -  HALL_KILL_ENABLE     :PB2
 *       HALL_ON_OFF   -  HALL_HARDKILL        :PB0
 *
 *  POSSIBLE BOARD LOCK FUNCTIONS:
 *  BOARD LOCKS WILL OCCUR UNDER KILL CONDITIONS(EITHER SOFT OR HARD)
 *  IN THIS, THE PROGRAM WILL BE LOCKED IN AN STATE WAITING TO BE UNKILLED
 *  THESE CAN ORIGINATE FROM HALL INPUTS OR VIA A KILL MESSAGE OVER CAN
 *
 *  CAN_Mobo_Unkill - will lock if motherboard transmits soft kill message
 *      must be unkilled by motherboard
 *
 *  TIM0_ISR - will lock if soft kill hall effect is removed
 *      must be unkilled by readding the magnet
 *
 *  TIM1_ISR - will lock if the system hard  kill hall magnet is removed
 *      must be unkilled by turning the sub on
 *
 *
 * NOTE CAN MESSAGES:
 * This board should recieve 3 different types of messages
 *
 * Task Groups: Thruster, Kill
 *
 * kill messages on the kill channel
 * thruster commands on thruster channel
 * heart beats on thruster channel
 *
 *
 * NOTE: MOTHERBOARD CANNONT TRANSMIT HARD KILL MESSAGES IN THIS VERSION OF THE FIRMWARE
 *
 */

//includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "MIL_BR_ESC.h" //ESC header
#include "MIL_CLK.h"
#include "MIL_CAN.h"
#include "Thruster_Kill_Board.h"

//defines

/*swap these depending on what voltage level corresponds to
 * no magnet
 *
 * if hall reading = false, it will kill sub
 */
#define HALL_TRUE  HALL_LO
#define HALL_FALSE HALL_HI

//heart beat limit
/*
 * indicates how many heart beats can be missed before we kill power
 * to thrusters
 */
#define HEART_LIMIT 100
#define IDLE_LIMIT 50

/*********************************************ISR PROTO*************************************************/

/*
 * ISR Tasks
 * Check SoftKill(will lock if happens)
 * Increment Heart Beat counter
 *
 */
void TIM0_ISR(void);

/*
 * ISR Task
 * CHeck HardKill/On_Off
 * Set check idle flag
 * Set check go flag
 */
void TIM1_ISR(void);

/*********************************************FXN PROTO*************************************************/

/*
 * Desc: This will parse data that results from a thruster commands from motherboard
 *       Confirm if it's a command message
 *
 *       if command
 *          extract thruster ID
 *          extract thrust value
 *          command thrusters
 *       else
 *          ignore
 */
void Thrust_Pack_Handler(uint8_t *pMsg,tkb_thrust_data_t *thrusters);

/*
 * Desc: This will parse data that results from a kill commands from motherboard
 *        and the rest of the CAN network
 *
 *       Confirm if it's a command message
 *       if command
 *          if soft
 *              run soft kill
 *       else
 *          ignore
 *
 * NOTE: MOTHERBOARD CANNONT TRANSMIT HARD KILL MESSAGES IN THIS VERSION OF THE FIRMWARE
 */
void Kill_Pack_Handler(uint8_t *pMsg,MIL_CAN_MailBox_t MailBox);


/*
 * Desc: Handles heart beat logic
 * If we've missed a heart beat 50
 * times(kept track of by counter)
 *
 * We cut power to thrusters
 *
 */
void HeartBeat_Handler(void);

/*
 * Desc: will return 0 if the motherboard has not
 *       sent unklll messsag
 */
uint8_t CAN_Mobo_Unkill(MIL_CAN_MailBox_t MailbBox);

/*
 * Desc: checks if the message is an unkill message
 * K - Kill
 * C - command
 * U - Unassert
 *
 * Returns 1 if true and 0 otherwise
 *
 */
uint8_t Check_MSGisKCSU(uint8_t *pMsg);


//FLAGS
//timer ISR designated to soft kill if no new data received
volatile uint8_t tim_softkill_flag = 0; //will be triggered by timer ISR

volatile uint8_t tim_idle = 0;
volatile uint8_t tim_go_check = 0;

//heartbeat flags
/*increments each time the timer0 interrupt runs, resets to 0 if heartbeat message received
  when the counter is > 50 (and timer0 set to 10 ms), over 500 ms have passed without heartbeat
  making the heartbeat lost flag become true*/
volatile uint8_t heartbeat_missed_counter = 0;
volatile uint8_t heartbeat_lost_flag = 0;      //if heart beat lost, set this

//overall softkill flag goes true if any of the concerned flags in kill check are true
volatile uint8_t hardkill_flag = 0;
volatile uint8_t overall_softkill_flag = 0;
volatile uint8_t overall_softkill_last = 0;
volatile uint8_t HALL_softkill_flag = 0;
volatile uint8_t mobo_softkill_flag = 0;

//initialize ESCs flags and timer for unkill subprocess
volatile uint8_t init_ESCS_start = 0;
volatile uint16_t init_ESCS_timer = 0;
volatile uint8_t init_ESCS_CMDsent = 0;
volatile uint8_t init_ESCS_done = 0;

//counts how many times the board tried to idle
volatile uint8_t idle_counter = 0;

//counts TIM_ISR0 loops to determine when to send messages to mobo, preventing spam
volatile uint8_t moboTxWait_counter = 0;

/*** CAN MESSAGES ***/
/* TX MESSAGES */
//C strings are terminated by Null character
//static const uint8_t C_KILL_LEN = 5;
//static const uint8_t C_GO_LEN = 3;
//static const uint8_t C_HEARTBEAT_LEN = 3;
static const uint8_t C_STATUS_LEN = 2;

//2 bytes containing the board's current kill and input status
//First byte: kill flag statuses (1 = kill raised, 0 = kill cleared)
//  7: Hard kill, 6: Overall soft kill, 5: Hall effect soft kill,
//  4: Mobo command soft kill, 3: Heartbeat lost flag, 2-0: Reserved for future kill sources
//Second byte: Input statuses and thruster initialization status (1 = "pressed")
//  7: On (1) / Off (0) Hall effect status,
//  6: Soft kill hall effect status, 5: Go hall effect status,
//  4: Thruster initialization status (1 if powering thrusters, 0 when done)
//  3-0: Reserved for future debugging status info
volatile uint8_t boardStatus[C_STATUS_LEN] = {0};
volatile bool lastGo = 0;

//static const char Hard_Killed[C_KILL_LEN]   = "KRHA";    // 0x4B 0x52 0x48 0x41 0x00
//static const char Soft_Killed[C_KILL_LEN]   = "KRSA";    // 0x4B 0x52 0x53 0x41 0x00
//static const char Hard_UnKilled[C_KILL_LEN] = "KRHU";    // 0x4B 0x52 0x48 0x55 0x00
//static const char Soft_UnKilled[C_KILL_LEN] = "KRSU";    // 0x4B 0x52 0x53 0x55 0x00
//static const char Go_Asserted[C_GO_LEN]     = "GA";      // 0x47 0x41 0x00
//static const char Go_UnAsserted[C_GO_LEN]   = "GU";      // 0x47 0x55 0x00

/* RX MESSAGES */
//static const char Hard_Killed_CMD[C_KILL_LEN]   = "KCHA";    // 0x4B 0x43 0x48 0x41 0x00
//static const char Soft_Killed_CMD[C_KILL_LEN]   = "KCSA";    // 0x4B 0x43 0x53 0x41 0x00
//static const char Hard_UnKilled_CMD[C_KILL_LEN] = "KCHU";    // 0x4B 0x43 0x48 0x55 0x00
//static const char Soft_UnKilled_CMD[C_KILL_LEN] = "KCSU";    // 0x4B 0x43 0x53 0x55 0x00
//static const char Heartbeat_CMD[C_HEARTBEAT_LEN] = "HM";    // 0x48 0x4D 0x00

uint8_t debug_softkill = 0;


/*********************************************MAIN*************************************************/

int main(void){

    MIL_ClkSetInt_16MHz();

    //LITERALLY EVERY PORT USED IN THIS DESIGN
    INIT_ALL_PORT_CLKS();

    /*********************************************IO INIT START*************************************************/

    //Initiazlize power to main
    //main is powered
    INIT_MAIN_PWR_IO();

    //Initialize the Kill IO
    //power to thrusters cut
    //until power on called(in the ESC init)
    INIT_THRUST_PWR_IO();

    /*
     * Initialize HALL inputs
     *
     * Activation level corresponds to which voltage
     * means unkill
     *
     * so if HALL_ACT_HI , then low will kill and high will unkill
     * if HALL_ACT_LO, then high will kill and low will unkil
     */
    Init_HALL_IO(HALL_ACT_LO);

    /*********************************************IO INIT END*************************************************/


    /********************************************DATA START***********************************************/



    /*********TRHUSTERS*************/
     //thruster attributes
    tkb_thrust_data_t thrust0_fhl = {.thrust_addr = 0,.pwm_gen = TKB_FH_PWM_GEN,.pwm_out = TKB_PWM_FHL_PIN,.speed.speed_float = 0},
                      thrust1_fhr = {.thrust_addr = 1,.pwm_gen = TKB_FH_PWM_GEN,.pwm_out = TKB_PWM_FHR_PIN,.speed.speed_float = 0},
                      thrust2_fvl = {.thrust_addr = 2,.pwm_gen = TKB_FV_PWM_GEN,.pwm_out = TKB_PWM_FVL_PIN,.speed.speed_float = 0},
                      thrust3_fvr = {.thrust_addr = 3,.pwm_gen = TKB_FV_PWM_GEN,.pwm_out = TKB_PWM_FVR_PIN,.speed.speed_float = 0},
                      thrust4_bhl = {.thrust_addr = 4,.pwm_gen = TKB_BH_PWM_GEN,.pwm_out = TKB_PWM_BHL_PIN,.speed.speed_float = 0},
                      thrust5_bhr = {.thrust_addr = 5,.pwm_gen = TKB_BH_PWM_GEN,.pwm_out = TKB_PWM_BHR_PIN,.speed.speed_float = 0},
                      thrust6_bvl = {.thrust_addr = 6,.pwm_gen = TKB_BV_PWM_GEN,.pwm_out = TKB_PWM_BVL_PIN,.speed.speed_float = 0},
                      thrust7_bvr = {.thrust_addr = 7,.pwm_gen = TKB_BV_PWM_GEN,.pwm_out = TKB_PWM_BVR_PIN,.speed.speed_float = 0};

    //a pointer array so I can address thruster by index if need be
    tkb_thrust_data_t pthrusters[8];
    pthrusters[0] = thrust0_fhl;
    pthrusters[1] = thrust1_fhr;
    pthrusters[2] = thrust2_fvl;
    pthrusters[3] = thrust3_fvr;
    pthrusters[4] = thrust4_bhl;
    pthrusters[5] = thrust5_bhr;
    pthrusters[6] = thrust6_bvl;
    pthrusters[7] = thrust7_bvr;

    /********CAN MAILBOXES*********/
        /*
         * Desc: Basically, the thruster cares about two separate message
         *       Thruster commands from the motherboard and kill commands
         *       from non mobo
         *
         *       The below mailboxes are Configured as so
         */

    //DECLARE BUFFERS THEN SET BUFFERS IN STRUCTS
    uint8_t Mobo_Data[TKB_CAN_MOBO_LEN];
    uint8_t Kill_Data[TKB_CAN_KILL_LEN];
     //all struct data initialized here
     MIL_CAN_MailBox_t CAN_KillBox = {.canid = TKB_KILLID, .filt_mask = TKB_KILL_FILTID_bm,.base = TKB_CAN_BASE,.msg_len = TKB_CAN_KILL_LEN,.obj_num = 1,.rx_flag_int = 0,.buffer = Kill_Data},
                       CAN_MoboBox = {.canid = TKB_MOBOID, .filt_mask = TKB_MOBO_FILTID_bm,.base = TKB_CAN_BASE,.msg_len = TKB_CAN_MOBO_LEN,.obj_num = 2,.rx_flag_int = 0,.buffer = Mobo_Data};




    /********************************************DATA END*************************************************/

    /**************************************THRUSTER INIT START********************/

    /*
     * Thruster init broken into 2 function
     * because the ESC init will be called
     * after a hard kill but not the pwm init
     */
    //will intialize the module
    TKB_PWM0_Init();

    //will send stop signal to ESCs
    //to begin communication
    TKB_Init_ESC();
    init_ESCS_done = 1;

    /**************************************THRUSTER INIT END**********************/

    /**************************************CAN INIT END**********************/

    //CHECK THRUSTER KILL BOARD SOURCE FILE FOR POSSIBLE MESSAGES
    /*
     * NOTE: for CAN communications, there are two separate
     *       mailboxes. One for thrust commands coming from
     *       motherboard and the other for kill events from the
     *       general CAN bus
     *
     * MB = Mail Box
     */

    TKB_CANInit();

    //initalize KILL mailbox
    MIL_InitMailBox(&CAN_KillBox);
    MIL_InitMailBox(&CAN_MoboBox);


    /**************************************CAN INIT START********************/

    /**************************************TIMER INIT START********************/

    //configured for 10ms period
    /*
     * This is done periodically in main to avoid
     * constantly soft killing the thrusters
     *
     * The period will get adjusted to what will
     * be considered the normal rate at which
     * motherboard will transmit messages
     */
    Timer0_OVF_Init(&TIM0_ISR,10);

    //configured for 1second period
    /*
     * CAN will periodically send Go status to motherboard
     * GO will be interpreted by higher level decision making
     *
     */
    Timer1_OVF_Init(&TIM1_ISR, 1000);


    /**************************************TIMER INIT START********************/


    /*********************************PRE-EMPTIVE HARD LOCK START**************/
    /*
     * DESC: IF EITHER KILL STATUS IS ASSERTED
     *       STALL HERE BEFORE EVEN ENTERING THE
     *       WHILE ONE LOOP
     *
     */

    //if the off is asserted, hard lock the system
    if(HALL_Check_ON_OFF() == HALL_FALSE){
        TIM1_ISR();
    }
    boardStatus[1] |= 0x80; //if not, set the on/off status flag to 1 (on)

    //if the Soft Kill is asserted, hard lock the system
    if(HALL_CheckSoftKill() == HALL_FALSE){
        debug_softkill = HALL_CheckSoftKill();
        TIM0_ISR();
    }

    /*********************************PRE-EMPTIVE HARD LOCK END****************/

    //master int enable
    IntMasterEnable();


    /*
     * Task list:
     *
     * Handle Messages from Kill Channel
     *
     * Handle Messages from Thruster Channel
     *
     * Report status of GO
     *
     * Handle Heart Beat Logic
     *
     */
    while(1){

        /****************CAN HANDLING START**************************/
        /*
         * Software implemetend priority encoder
         */

        //Check for non motherobard sources of kill
        if(MIL_CAN_CheckMail(&CAN_KillBox)== MIL_CAN_OK){
            MIL_CAN_GetMail(&CAN_KillBox);

            //if it's a kill message confirmed,run the kill handler
            //the kill lock
            if(TKB_Check_KillMsg(Kill_Data)){
                Kill_Pack_Handler(Kill_Data,CAN_KillBox);
            }

            //if it's a heart beat, reset missed counter
            else if(TKB_Check_HeartbeatMsg(Kill_Data)){
                heartbeat_missed_counter = 0;
            }

            else
            {
                init_ESCS_start = 0;
            }

            //expand here to accept more messages from kill channel

        }

        //interpret motherboard data
        else if(MIL_CAN_CheckMail(&CAN_MoboBox)== MIL_CAN_OK){
            MIL_CAN_GetMail(&CAN_MoboBox);
            idle_counter = 0x00; //set idle command to 0 since we received something

            // If the overall_softkill_flag is true, do not process any further

            //if it's a thruster message do this
            if((TKB_Check_ThrustMsg(Mobo_Data)) &&
               (overall_softkill_flag == 0)         &&
               (init_ESCS_done == 1)){
                Thrust_Pack_Handler(Mobo_Data,pthrusters);
            }

            //expand here to accept more messages from thruster channel

        }
        //systems will periodically ensure the thrusters are idle if there hasn't been
        //a message sent from motherboard
        else{
            if(idle_counter >= IDLE_LIMIT){
                TKB_IdleThrusters();
            }
        }
        /****************CAN HANDLING END**************************/

        /**************HEART BEAT CHECK START*************************************/

        HeartBeat_Handler();

        /**************HEART BEAT CHECK END***************************************/

        /**************GO CHECK AT THE END OF EACH LOOP******************/
        if(tim_go_check){
            //check go status and decide message
            //if not asserted
             if(HALL_CheckGo() == HALL_FALSE){
                 //transmit status
                 if(lastGo == 1)
                 {
                     boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                             HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
                     boardStatus[1] &= 0xDF;
                     moboTxWait_counter = 0;
                     MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);
                     lastGo = 0;
                 }
             }
             else{
                 //transmit status
                 if(lastGo == 0)
                 {
                     boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                             HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
                     boardStatus[1] |= 0x20;
                     moboTxWait_counter = 0;
                     MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);
                     lastGo = 1;
                 }
             }
             tim_go_check = 0;
        }

        /**************GO CHECK END***************************************/

        /**************OVERALL KILL CHECK START***********************************/

        if(HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag)
        {
            overall_softkill_flag = 0x40;
        }
        else
        {
            overall_softkill_flag = 0x00;
        }

        // This kill soft kills for any of the triggering kill flags.
        // On/Off should execute a special function, but could still run this
        if(overall_softkill_flag)
        {
            if(overall_softkill_last == 0)
            {
                // Send once immediately on kill, repeats every second in TIM0_ISR
                TKB_SoftKill();
                boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                        HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
                boardStatus[1] &= 0xEF; // Sets the "thrusters initializing" flag to 0
                moboTxWait_counter = 0;
                MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);

                init_ESCS_start = 0;
                init_ESCS_done = 0;
            }
            overall_softkill_last = 1;
        }
        else
        {
            if(overall_softkill_last == 1)
            {
                // This runs the unkill after all concerned kill flags are zeroed, only once,
                // only if the overall kill previously ran the kill command
                // The unkill here sets the flag to begin the ESC initialization subprocess
                // instead of using TKB_UnKill() so that communication with the motherboard
                // is uninterrupted

                //TKB_UnKill();
                init_ESCS_start = 1;
                init_ESCS_timer = 0;
                init_ESCS_CMDsent = 0;
                init_ESCS_done = 0;

                POWER_MAIN();
                POWER_THRUSTERS();

                boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                        HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
                boardStatus[1] |= 0x10; // Sets the "thrusters initializing" flag to 1
                moboTxWait_counter = 0;
                MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);
            }
            overall_softkill_last = 0;
        }

        /**************OVERALL KILL CHECK END*************************************/

        /**************INITIALIZE ESCS SUBPROCESS START***************************/

        if(init_ESCS_start == 1)
        {
            // Wait 300 * 10 ms for the thrusters to power up,
            // then send the stop command to the thrusters to initialize them.
            // This runs only once, and then sets the init_ESCS_CMDsent flag
            if((init_ESCS_CMDsent == 0) && (init_ESCS_timer >= 300))
            {
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_FHL_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_FHR_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_FVL_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_FVR_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_BHL_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_BHR_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_BVL_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));
                PWMPulseWidthSet(TKB_PWM_BASE,
                                 TKB_PWM_BVR_PIN,
                                 PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));

                TKB_PWM_OUT_EN();
                init_ESCS_CMDsent = 1;
                init_ESCS_timer = 0;
            }

            if((init_ESCS_CMDsent == 1) && (init_ESCS_timer >= 200))
            {
                // After sending the stop command and waiting 300 * 10 ms,
                // set the init_ESCS_done flag to allow sending thrust commands.
                // End the subprocess by resetting the previous flags
                init_ESCS_start = 0;
                init_ESCS_CMDsent = 0;
                init_ESCS_timer = 0;
                init_ESCS_done = 1;
                boardStatus[1] &= 0xEF; // Sets the "thrusters initializing" flag to 0 (done)

                // Notify the mobo that the unkill process is complete
                boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                        HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
                moboTxWait_counter = 0;
                MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);
            }
        }

        /**************INITIALIZE ESCS SUBPROCESS END*****************************/

    }

}

/*********************************************FUNC DEFINITIONS**********************************/

/*
 * Desc: This will parse data that results from a thruster commands from motherboard
 *
 *          extract thruster ID
 *          extract thrust value
 *          command thrusters
 *
 * Parameters:
 * thrusters - our set indexable array of thrusters object
 * pMsg- pointer to received message
 */
void Thrust_Pack_Handler(uint8_t *pMsg,tkb_thrust_data_t *pthrusters){

    uint8_t thrust_id = pMsg[THRUST_ID_IDX];

     //extract float data
     for(uint8_t i = 0;i < 4 ;i++){

         pthrusters[thrust_id].speed.array[i] = pMsg[THRUST_FLOAT_START + i];

      }
      TKB_PWM_SetSpeed(pthrusters[thrust_id]);

}

/*
 * Desc: This will parse data that results from a kill commands from motherboard
 *        and the rest of the CAN network
 *
 *       Confirm if it's a command message
 *       if command
 *          if soft
 *              run soft kill
 *
 *       else
 *          ignore
 *
 * LOCK: THIS FUNCTION WILL LOCK THE BOARD UNTIL MOBO UNKILLS IT
 *       IF LOCK CONDITION MET
 *
 * NOTE: MOTHERBOARD CANNONT TRANSMIT HARD KILL MESSAGES IN THIS VERSION OF THE FIRMWARE
 */
void Kill_Pack_Handler(uint8_t *pMsg,MIL_CAN_MailBox_t MailBox){

    //used within function to unkill if a kill lock occured
//    uint8_t killed_flag = 0;

    //only react if it's a command byte and it's telling it to assert
    if((pMsg[MSG_CR_IDX] == CMD_BYTE) && (pMsg[MSG_UA_IDX] == A_BYTE) ){

        //check if hard or soft
        if(pMsg[KILL_TYPE_IDX] == SOFT_BYTE){
//            TKB_SoftKill();
//
//            //lock and transmit lock status
//            //if you haven't received an unkill message
//            while(!CAN_Mobo_Unkill(MailBox)){
//
//                MS_100_DELAY();
//
//            }
//            killed_flag = 1;

            mobo_softkill_flag = 0x10;
        }
//        else if(pMsg[KILL_TYPE_IDX] == HARD_BYTE){
//           IntMasterDisable();
//           TKB_HardKill();
//           kill_flag = 1;
//        }
    }
    else if(Check_MSGisKCSU(pMsg))
    {
        mobo_softkill_flag = 0x00;
    }

//    if(killed_flag){
//        TKB_UnKill();
//        killed_flag = 0;
//    }
}


/*
 * Desc: Handles heart beat logic
 * If we've missed a heart beat 50
 * times(kept track of by counter)
 *
 * We cut power to thrusters
 *
 */
void HeartBeat_Handler(void){

    //set the flag if we've missed 100 timer beats
    if(heartbeat_missed_counter > HEART_LIMIT){
        heartbeat_lost_flag = 0x08;
    }
    else{
        heartbeat_lost_flag = 0x00;
    }

//    //if the flag was previously lost and the counter hasn't been
//    //reset by the heart beat handler
//    if(heartbeat_lost_flag){
//
//        KILL_THRUSTERS();
//
//    }
//
//    /*
//     * If the heart beat flag we previously asserted and the counter was previously reset
//     * this means we lost heart beat and have found it again(counter being reset by receiving message)
//     */
//    else if(heartbeat_lost_flag && (heartbeat_missed_counter < HEART_LIMIT)){
//
//        TKB_UnKill();
//        heartbeat_lost_flag = 0;
//
//    }

}



/*
 * Desc: will return 0 if the motherboard has not
 *       sent unklll messsag
 */
//uint8_t CAN_Mobo_Unkill(MIL_CAN_MailBox_t MailbBox){
//
//        /*
//         * check for new message
//         * determine if it's a KCU(Kill command unassert)
//         */
//        if(MIL_CAN_CheckMail(&MailbBox) == MIL_CAN_OK){
//
//            MIL_CAN_GetMail(&MailbBox);
//
//                //unlock if correct message recieved
//                if(Check_MSGisKCSU(MailbBox.buffer)){
//                   return 0x01;
//                }
//                //otherwise stay locked
//                else{
//                    //transmit lock
//                    MIL_CANSimpleTX(TKB_CANID,Soft_Killed,C_KILL_LEN,TKB_CAN_BASE);
//                }
//        }
//        return 0x00;
//
//}

/*
 * Desc: checks if the message is an unkill message
 * K - Kill
 * C - command
 * U - Unassert
 *
 * Returns 1 if true and 0 otherwise
 */
uint8_t Check_MSGisKCSU(uint8_t *pMsg){

    if((pMsg[MSG_TYPE_IDX]  == KILL_START_BYTE) &&
       (pMsg[MSG_CR_IDX]    == CMD_BYTE)        &&
       (pMsg[KILL_TYPE_IDX] == SOFT_BYTE)       &&
       (pMsg[MSG_UA_IDX]    == U_BYTE)){
        return 1;
    }
    else{return 0;}

}




/*********************************************FUNC DEFINITIONS**********************************/

//tell system to soft kill
//if softkill magnet has been removed
/*
 * IF SOFT KILL CONFIRMED, PROGRAM WILL GET LOCKED IN THIS ISR
 */
void TIM0_ISR(void){

//    uint8_t kill_flag = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //if the Soft Kill is asserted, hard lock the system
    if (HALL_CheckSoftKill() == HALL_FALSE)
    {
        HALL_softkill_flag = 0x20;
        boardStatus[1] &= 0xBF;
    }
    else
    {
        HALL_softkill_flag = 0x00;
        boardStatus[1] |= 0x40;
    }
//    while(HALL_CheckSoftKill() == HALL_FALSE){
//        TKB_SoftKill();
//        MIL_CANSimpleTX(TKB_CANID,Soft_Killed,C_KILL_LEN,TKB_CAN_BASE);
//        kill_flag = 1;
//    }
//
//    if(kill_flag){
//        TKB_UnKill();
//        kill_flag = 0;
//    }

    //send a soft kill asserted response message every 1 second
    // TODO: Possibly make the response message identify what is triggering kill
    if(moboTxWait_counter > 100)
    {
        boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
        MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);
        moboTxWait_counter = 0;
    }
    else
    {
        moboTxWait_counter += 1;
    }

    //increment the missed heartbeat counter (resets if a heartbeat message is received)
    if(heartbeat_lost_flag == 0)
    {
        heartbeat_missed_counter += 1;
    }

    //increment the init_ESCS_timer if the init_ESCS_start flag is set
    if(init_ESCS_start == 1)
    {
        init_ESCS_timer += 1;
    }

    //increment idle counter
    idle_counter++;

}

//transmits go status
//also checks for hardkill/on_off
/*
 * IF HARD KILL CONFIRMED, PROGRAM WILL GET LOCKED IN THIS ISR
 */
void TIM1_ISR(void){

    //used within function to unkill if a kill lock occured
    uint8_t kill_flag = 0;
    TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    while(HALL_Check_ON_OFF() == HALL_FALSE){
        IntMasterDisable();
        hardkill_flag = 0x80;
        boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
        boardStatus[1] &= 0x7F;
        MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);

        TKB_HardKill();
        kill_flag = 1;
    }

    if(kill_flag){
        IntMasterEnable();
        hardkill_flag = 0x00;
        boardStatus[0] = (hardkill_flag | overall_softkill_flag |
                HALL_softkill_flag | mobo_softkill_flag | heartbeat_lost_flag);
        boardStatus[1] |= 0x80;
        MIL_CANSimpleTX(TKB_CANID,boardStatus,C_STATUS_LEN,TKB_CAN_BASE);

        TKB_UnKill();
        kill_flag = 0;
    }

    //set idle flags
    tim_idle = 0xFF;

    //check go input
    tim_go_check = 0xFF;

}
