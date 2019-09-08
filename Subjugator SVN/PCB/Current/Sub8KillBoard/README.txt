Sub 8 Kill Board Documentation

Created: 1/10/2019
Updated: 5/13/2019

The hardware for this board consists of 4 major sections: Power, Command, Communication, and Kill.
The software for this board cosists of embedded code on the MCU that will interact with the main sub 
computer via CAN.

----- Hardware -----
The power section uses the standard 20V <= Vin <= 30V input with a 5V 1A output, as well as a 3.3V LDO regulator.
Currently the power from the sub comes from 2 separate 24V batteries (in parallel) into the sherlock connector
in this schematic. This 24V is linked to the connectors in the kill section for providing direct power to the
motors. This voltage is also regulated down to 5V and again to 3.3V for logical use.

The command section houses the MCU (TM4C123GH6PM). This sheet also contains an external connector for
supplying 3.3V and GND for debugging purposes, as well as the programmer connections, a power and debug LED,
and a reset switch. Information on the pin/port connections and the modules in use can be found in the
schematic.

The communication section is the home of CAN, the Hall Effect Sensors, and the PWM outputs for the ESCs. This
schematic mainly consists of sherlock connectors. There is an added connector for UART capabilities if needed,
which shares the same pin/port configuration as CAN. There is one connector for each thrusters PWM control,
which is used to send commands to the ESC for the respective Blue Robotics T200 Thruster. There are also
connectors for each Hall Effect sensor, and their uses are described below:
Kill Enable - Used to start the robot at competitions (GO Command)
Soft Kill - Software Kill, no commands are sent to the ESC's and power to the thursters IS NOT cut
Hard Kill - Power Kill, all attributes of the soft kill but power IS cut using relays

The kill section schematic holds the sherlock connectors for each motor pair relay, and the MOSFETs
which will all be controlled via the respective signal. Any given motor will have its front-back pair
connected to a relay. If the control signal is true, then the MOSFETs and motor relays will open to 
prevent the motors connected to that relay from receiving power. There is also an on/off relay for
controlling power to the sub (this board and power merge will always be powered). A flywheel diode is
added to prevent thruster malfunction. This is also where the connectors carrying the 24V to the motors
are located.


----- Software -----

The software intended to run on this board is only the code flashed onto the MCU which will handle the status of
the Hall Effect Sensors, the various motor PWMs, and the KILL Signal. Other Sub 8 software will interact with this
board via the CAN (or possibly UART) which will help guide the MCU to the desired functionality.