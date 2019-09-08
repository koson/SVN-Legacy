#include "pch.h"
#include <iostream>
using namespace std;

unsigned int sof = 0xC0;
unsigned int eof = 0xC1;
unsigned int R_nT;
unsigned int checksum;
unsigned int data_length;
unsigned int can_filter_id;
unsigned int num;
unsigned int *dataBuffer;

int main() {
	while (1) {
		cout << "**********************USB-to-CAN Command Generator**********************";
		checksum = 0;
		cout << endl << "Enter R_nt_ENABLE: ";
		cin >> R_nT;
		//cout << endl;

		cout << "Enter Data Length: ";
		cin >> hex >> data_length;
		//cout << endl;

		cout << "Enter Filter ID in Hex: ";
		cin >> hex >> can_filter_id;
		//cout << endl;

		dataBuffer = new unsigned int[data_length + 4];
		dataBuffer[0] = sof;
		dataBuffer[1] = (R_nT << 7) | (data_length - 1);
		dataBuffer[2] = can_filter_id;
		dataBuffer[data_length + 3] = eof;

		if (R_nT) {
			data_length = 0;
			dataBuffer[data_length + 3] = eof;
		}

		for (int i = 3; i < 3 + data_length; i++) {
			cout << "Enter Byte " << i << ":";
			cin >> hex >> dataBuffer[i];
			cout << endl;
		}

		for (int i = 0; i < data_length + 4; i++) {
			checksum += dataBuffer[i];
		}
		checksum = checksum % 16;
		cout << "Checksum: " << checksum << endl;

		cout << "USB-to-CAN Command: ";
		dataBuffer[1] |= (checksum << 3);
		for (int i = 0; i < data_length + 4; i++) {
			if (i == data_length + 4 - 1) {
				cout << hex << "0x" << dataBuffer[i] << endl;
			}
			else {
				if (dataBuffer[i] < 0x10) {
					cout << hex << "0x0" << dataBuffer[i] << ",";
				}
				else {
					cout << hex << "0x" << dataBuffer[i] << ",";
				}
			}
		}

		/*for (int i = 0; i < data_length + 4; i++) {
			if (i == data_length + 4 - 1) {
				cout << hex << "0x" << dataBuffer[i] << endl;
			}
			else {
				if (dataBuffer[i] < 0x10) {
					cout << hex << "0x0" << dataBuffer[i] << "+";
				}
				else {
					cout << hex << "0x" << dataBuffer[i] << "+";
				}
			}
		}*/
		cout << "************************************************************************" << endl << endl;
	}
}