/*
 * Name: Thruster_Kill_Board (TKB)
 * Author: Marquez Jones
 * Date Started: 3/22/19
 * Desc: Kill board thruster spin test
 *       All it does is initialize ESCs and set all thrusters
 *       to 0.5 speed
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
#define HALL_TRUE  HALL_HI
#define HALL_FALSE HALL_LO

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
void Kill_Pack_Handler(uint8_t *pMsg,MIL_CAN_MailBox_t MoboBox);


/*
 * Desc: will return 0 if the motherboard has not
 *       sent unklll messsag
 */
void CAN_Mobo_Unkill(MIL_CAN_MailBox_t MoboBox);

/*
 * Desc: checks if the message is an unkill message
 * K - Kill
 * C - command
 * U - Unassert
 *
 * Returns 1 if true and 0 otherwise
 *
 */
uint8_t Check_MSGisKCU(uint8_t *pMsg);


//FLAGS
//timer ISR designated to soft kill if no new data received
uint8_t tim_softkill_flag = 0; //will be triggered by timer ISR

uint8_t tim_idle_check = 0;
uint8_t tim_go_check = 0;

/*** CAN MESSAGES ***/
/* TX MESSAGES */
//C strings are terminated by Null character
static const uint8_t C_KILL_LEN = 5;
static const uint8_t C_GO_LEN = 3;
static const char Hard_Killed[C_KILL_LEN]   = "KRHA";    // 0x4B 0x52 0x48 0x41 0x00
static const char Soft_Killed[C_KILL_LEN]   = "KRSA";    // 0x4B 0x52 0x53 0x41 0x00
static const char Hard_UnKilled[C_KILL_LEN] = "KRHU";    // 0x4B 0x52 0x48 0x55 0x00
static const char Soft_UnKilled[C_KILL_LEN] = "KRSU";    // 0x4B 0x52 0x53 0x55 0x00
static const char Go_Asserted[C_GO_LEN]     = "GA";      // 0x47 0x41 0x00
static const char Go_UnAsserted[C_GO_LEN]   = "GU";      // 0x47 0x55 0x00

/* RX MESSAGES */
static const char Hard_Killed_CMD[C_KILL_LEN]   = "KCHA";    // 0x4B 0x43 0x48 0x41 0x00
static const char Soft_Killed_CMD[C_KILL_LEN]   = "KCSA";    // 0x4B 0x43 0x53 0x41 0x00
static const char Hard_UnKilled_CMD[C_KILL_LEN] = "KCHU";    // 0x4B 0x43 0x48 0x55 0x00
static const char Soft_UnKilled_CMD[C_KILL_LEN] = "KCSU";    // 0x4B 0x43 0x53 0x55 0x00

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
     //all struct data initialized here
     MIL_CAN_MailBox_t CAN_KillBox = {.canid = TKB_KILLID, .filt_mask = TKB_KILL_FILTID_bm,.base = TKB_CAN_BASE,.msg_len = TKB_CAN_KILL_LEN,.obj_num = 1,.rx_flag_int = 0},
                       CAN_MoboBox = {.canid = TKB_MOBOID, .filt_mask = TKB_MOBO_FILTID_bm,.base = TKB_CAN_BASE,.msg_len = TKB_CAN_MOBO_LEN,.obj_num = 2,.rx_flag_int = 0};

     //our data buffers
     uint8_t Kill_Data[TKB_CAN_KILL_LEN];
     uint8_t Mobo_Data[TKB_CAN_MOBO_LEN];


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

    /**************************************THRUSTER INIT END**********************/

    /**************************************CAN INIT START********************/

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

    /**************************************CAN INIT END********************/

    while(1){


        for(uint8_t i = 0;i<8;i++){

            pthrusters[i].speed.speed_float = 0.5;
            TKB_PWM_SetSpeed(pthrusters[i]);

        }


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
void Kill_Pack_Handler(uint8_t *pMsg,MIL_CAN_MailBox_t MoboBox){

    //used within function to unkill if a kill lock occured
    uint8_t kill_flag = 0;

    //only react if it's a command byte and it's telling it to assert
    if((pMsg[MSG_CR_IDX] == CMD_BYTE) && (pMsg[MSG_UA_IDX] == A_BYTE) ){

        //check if hard or soft
        if(pMsg[KILL_TYPE_IDX] == SOFT_BYTE){

            IntMasterDisable();
            TKB_SoftKill();
            CAN_Mobo_Unkill(MoboBox);
            uint8_t kill_flag = 1;
        }
    }

    if(kill_flag){
        IntMasterEnable();
        TKB_UnKill();
        kill_flag = 0;
    }
}

/*
 * Desc: will return 0 if the motherboard has not
 *       sent unklll messsag
 */
void CAN_Mobo_Unkill(MIL_CAN_MailBox_t MoboBox){

    uint8_t lock_flag = 1; //will lock program in this functiong
    uint8_t *unlock_buffer; //will hold received CAN messages

    //until kill unasserted
    while(lock_flag){


        /*
         * check for new message
         * determine if it's a KCU(Kill command unassert)
         */
        if(MIL_CAN_CheckMail(&MoboBox) == MIL_CAN_OK){

            MIL_CAN_GetMail(unlock_buffer, &MoboBox);

                //unlock if correct message recieved
                if(Check_MSGisKCU(unlock_buffer)){
                    lock_flag = 0;
                }
                //otherwise stay locked
                else{
                    lock_flag = 1;
                    //transmit lock
                    MIL_CANSimpleTX(TKB_CANID,Soft_Killed,C_KILL_LEN,TKB_CAN_BASE);
                    MS_100_DELAY(); //to avoid spamming the transmit, just soft delay 100ms
                }
        }
        else{
            lock_flag = 0;
        }
    }
}

/*
 * Desc: checks if the message is an unkill message
 * K - Kill
 * C - command
 * U - Unassert
 *
 * Returns 1 if true and 0 otherwise
 */
uint8_t Check_MSGisKCU(uint8_t *pMsg){

    if(pMsg[MSG_TYPE_IDX] == KILL_START_BYTE &&
       pMsg[MSG_CR_IDX]   == CMD_BYTE        &&
       pMsg[MSG_UA_IDX]   == U_BYTE){
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
void TIM0_CheckSoftKill_ISR(void){

    uint8_t kill_flag = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //if the Soft Kill is asserted, hard lock the system
    while(HALL_CheckSoftKill() == HALL_FALSE){
        IntMasterDisable();
        TKB_SoftKill();
        kill_flag = 1;

    }

    if(kill_flag){
        IntMasterEnable();
        TKB_UnKill();
        kill_flag = 0;
    }

}

//transmits go status
void TIM1_Check_ON_OFF_ISR(void){

    //used within function to unkill if a kill lock occured
    uint8_t kill_flag = 0;
    TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    while(HALL_Check_ON_OFF() == HALL_FALSE){
        IntMasterDisable();
        TKB_HardKill();
        kill_flag = 1;
    }

    if(kill_flag){
        IntMasterEnable();
        TKB_UnKill();
        kill_flag = 0;
    }

    //set idle flags
    tim_idle_check = 0xFF;

    //check go input
    tim_go_check = 0xFF;

}



