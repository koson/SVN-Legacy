#include <USBtoCAN.h>

int main () {
    cout << "Hello World" << endl;
    InitUSBtoCAN();
    USBtoCAN_RUN();
    return 0;
}