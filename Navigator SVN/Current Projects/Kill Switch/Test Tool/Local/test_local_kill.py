import serial
import threading

ser = serial.Serial(port='COM25', baudrate=9600, timeout=0.25)

def menu():
    while True:
        if raw_input() == '':
            print "Options:"
            print "    (0) Exit Menu"
            print "    (1) Ping"
            print "    (2) Query Parameters"
            print "    (3) Set Computer Kill State"
            print "    (4) Set Operation State"
            print "    (5) Set Horn State"
            print "    (Ctrl-C) Exit Program"
            option = input("Selection: ")
            if option == 0:
                print "Waiting... (Press enter to enter menu)"
            elif option == 1:
                ser.write('\x20')
                if ser.read() == '\x30':
                    print "OK"
                else:
                    print "Timeout"
            elif option == 2:
                print "Overall Kill Status",
                ser.write('\x21')
                response = ser.read()
                if response == '\x00':
                    print "Unkilled"
                elif response == '\x01':
                    print "Killed"
                else:
                    print "Timeout"
                    
                print "PF Status",
                ser.write('\x22')
                response = ser.read()
                if response == '\x00':
                    print "Released"
                elif response == '\x01':
                    print "Pressed"
                else:
                    print "Timeout"
                    
                print "PA Status",
                ser.write('\x23')
                response = ser.read()
                if response == '\x00':
                    print "Released"
                elif response == '\x01':
                    print "Pressed"
                else:
                    print "Timeout"
                    
                print "SF Status",
                ser.write('\x24')
                response = ser.read()
                if response == '\x00':
                    print "Released"
                elif response == '\x01':
                    print "Pressed"
                else:
                    print "Timeout"
                    
                print "SA Status",
                ser.write('\x25')
                response = ser.read()
                if response == '\x00':
                    print "Released"
                elif response == '\x01':
                    print "Pressed"
                else:
                    print "Timeout"
                    
                print "Remote Kill Status",
                ser.write('\x26')
                response = ser.read()
                if response == '\x00':
                    print "Unkilled"
                elif response == '\x01':
                    print "Killed"
                else:
                    print "Timeout"
                    
                print "Computer Kill Status",
                ser.write('\x27')
                response = ser.read()
                if response == '\x00':
                    print "Unkilled"
                elif response == '\x01':
                    print "Killed"
                else:
                    print "Timeout"
            elif option == 3:
                mode = input("(1) Killed, (2) Unkilled: ")
                if mode == 1:
                    ser.write('\x45')
                    if ser.read() == '\x55':
                        print "OK"
                    else:
                        print "Timeout"
                elif mode == 2:
                    ser.write('\x46')
                    if ser.read() == '\x56':
                        print "OK"
                    else:
                        print "Timeout"
                else:
                    print "Invalid mode"
            elif option == 4:
                mode = input("(1) No Operation, (2) Manual Operation, (3) Autonomous Operation: ")
                if mode == 1:
                    ser.write('\x40')
                    if ser.read() == '\x50':
                        print ""
                    else:
                        print "Timeout"
                elif mode == 2:
                    ser.write('\x41')
                    if ser.read() == '\x51':
                        print "OK"
                    else:
                        print "Timeout"
                elif mode == 3:
                    ser.write('\x42')
                    if ser.read() == '\x52':
                        print "OK"
                    else:
                        print "Timeout"
                else:
                    print "Invalid mode"
            elif option == 5:
                mode = input("(1) Horn Off, (2) Horn On: ")
                if mode == 1:
                    ser.write('\x43')
                    if ser.read() == '\x53':
                        print "OK"
                    else:
                        print "Timeout"
                elif mode == 2:
                    ser.write('\x44')
                    if ser.read() == '\x54':
                        print "OK"
                    else:
                        print "Timeout"
                else:
                    print "Invalid mode"
            else:
                print "Unsupported Option"

print "Local Kill Test Tool"

print "Using serial resource:", ser.name

print "Waiting... (Press enter to enter menu)"

thread = threading.Thread(target = menu)
thread.start()

while True:
    if ser.inWaiting():
        response = ser.read()
        if response == '\x10':
            print "Notification: Overall Killed"
        elif response == '\x11':
            print "Notification: Overall Unkilled"
        elif response == '\x12':
            print "Notification: PF Pressed"
        elif response == '\x13':
            print "Notification: PF Released"
        elif response == '\x14':
            print "Notification: PA Pressed"
        elif response == '\x15':
            print "Notification: PA Released"
        elif response == '\x16':
            print "Notification: SF Pressed"
        elif response == '\x17':
            print "Notification: SF Released"
        elif response == '\x18':
            print "Notification: SA Pressed"
        elif response == '\x19':
            print "Notification: SA Released"
        elif response == '\x1A':
            print "Notification: Remote Killed"
        elif response == '\x1B':
            print "Notification: Remote Unkilled"
        elif response == '\x1C':
            print "Notification: Computer Killed"
        elif response == '\x1D':
            print "Notification: Computer Unkilled"

# if option == 1:
    # channel = input("Enter channel (0-100): ")
    # if channel >= 0 and channel <= 100:
        # ser.write('\xAD')
        # ser.write(struct.pack('>B', channel))
        # print "Channel set to:", int(ser.read().encode('hex'), 16)
    # else:
        # print "Invalid channel"
# elif option == 2:
    # ser.write('\xAC')
    # rssi = ser.read().encode('hex')
    # print "RSSI (hex)", rssi
    # print "RSSI (dBm)", int(rssi, 16) - 256
