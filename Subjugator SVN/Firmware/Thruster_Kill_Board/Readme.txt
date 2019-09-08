Project: Thruster/Kill Board(TKB)
Hardware Des By: Frank Mitchell
Firmware Des By: Marquez Jones
Desc: Firmware for board and other test firmware are located here

File Break down:
	CAN_Tx_Test_Bench: Test node that sends messages to the TKB
	Thruster_Kill    : File contains application code to be put on the board
	Thruster_Kill_Rx_Test: Isolated code to test CAN reception
	Thruser_Spin_Test : Isolated code to test thruster control

NOTE: ALL CODE REQUIRES TIVAWARE DRIVERS WHICH ARE NOT INCLUDED IN THE FILES.