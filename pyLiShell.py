# -------------------------------------------------------------------------------
# Application:  pyLiShell.py
# Author:       jwc
# Purpose:      A shell for accessing the Lithium-1 radio from Astrodev.
#               Compatible with protocole from 3.10 release from AD.
# Version:      0.1
# Date:         04 August 2016
# Todo:         - Commands to implement
#               xx Open serial port
#               xx Set TX frequency.
#               xx Set RX frequency.
#               xx Query settings.
#               xx Display settings.
#               xx Query telem (RSSI, packets rx/tx)
#               xx Display telem (RSSI, packets rx/tx)
#               xx Ping...to get an  ack back.
#               xx Parse ack
#               xx Check checksums
#               xx Add code to send over serial port
#               xx Add code to rx over port
#               -- TCP socket for TX/RXing data.  Lithium packet headers stripped and not added, just the raw data is sent through.  Bytes sent when a full packet is ready, that is payload is 255 bytes.
#               -- Double check against known software
#               - Test all commands!
#               - Add error checking.  :/
# Notes:        -
# -------------------------------------------------------------------------------

import sys, socket, cmd, time

def intToByteArray(value, size):
    b = bytearray(size)
    # print("intToByteArray: ",value)
    for i in range (0,size):
        offset = (size - 1 - i) * 8
        b[i] = (value>>offset) & 0xFF
    # Note the shift above shoudl be unsigned.  In Java, this is
    # >>>, but according to some online docs, this isn't
    # applicable (sign/unsigned) in Python.
    #print (''.join('{:02x}'.format(x) for x in b))
    return b

def printBytes( data ):
    print (' '.join('{:02x}'.format(x) for x in data))


class lithium():
    # #defines for Lithium
    IF_BAUD = ['9600', '19200', '38400', '76800', '115200']
    RF_BAUD = ['1200', '9600', '19200', '38400' ]
    MODULATION = ['GFSK', 'AFSK', 'BPSK']
    NO_OP_COMMAND = 0x01
    RESET_SYSTEM = 0x02
    TRANSMIT_DATA = 0x03
    RECEIVE_DATA = 0x04
    GET_TRANSCEIVER_CONFIG = 0x05
    SET_TRANSCEIVER_CONFIG = 0x06
    TELEMETRY_QUERY = 0x07
    WRITE_FLASH = 0x08
    RF_CONFIG = 0x09
    BEACON_DATA = 0x10
    BEACON_CONFIG = 0x11
    READ_FIRMWARE_REVISION = 0x12
    WRITE_OVER_AIR_KEY = 0x13
    FIRMWARE_UPDATE = 0x14
    FIRMWARE_PACKET = 0x15
    FAST_PA_SET = 0x20
    BAUD_RATE_9600 = 0
    BAUD_RATE_19200 = 1
    BAUD_RATE_38400 = 2
    BAUD_RATE_76800 = 3
    BAUD_RATE_115200 = 4
    RF_BAUD_RATE_1200 = 0
    RF_BAUD_RATE_9600 = 1
    RF_BAUD_RATE_19200 = 2
    RF_BAUD_RATE_38400 = 3
    RF_MODULATION_GFSK = 0
    RF_MODULATION_AFSK = 1
    RF_MODULATION_BPSK = 2
    TELEMETRY_DUMP_COMMAND = 0x30
    PING_RETURN_COMMAND = 0x31
    CODE_UPLOAD_COMMAND = 0x32
    RADIO_RESET_COMMAND = 0x33
    PIN_TOGGLE_COMMAND = 0x34

    def __init__(self):
        # Create Telemetry array

        # telemetryBytes = array('B',[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        self.telemetryBytes = bytearray(16)
        #   Telem description in code.
        #   typedef struct telem_type
        #   {
        #      0    uint_2 op_counter;
        #      2    sint_2 msp430_temp;
        #      4    uint_1 time_count[3];
        #      7    uint_1 rssi;
        #      8    uint_4 bytes_received;
        #      12   uint_4 bytes_transmitted;
        #   } TELEMETRY_STRUCTURE_type;

        self.radioConfiguration = bytearray(34)
        # Radio Configuration
        #   typedef struct {
        #   0   uint_1 interface_baud_rate;     // Radio interface baud rate
        #   1   uint_1 tx_power_amp_level;      // Tx Power Amp level(min=0x00 max = 0xFF)
        #   2   uint_1 rx_rf_baud_rate;         // Radio RX RF Baud Rate(9600 = 0x00)
        #   3   uint_1 tx_rf_baud_rate;         // Radio TX RF Baud Rate(9600 = 0x00)
        #   4   uint_1 rx_modulation;           // (0x00 = GFSK);
        #   5   uint_1 tx_modulation;           // (0x00 = GFSK);
        #   6   uint_4 rx_freq;                 // Channel Rx Frequency(ex: 45000000)
        #   10  uint_4 tx_freq;                 // Channel Tx Frequency(ex: 45000000)
        #   14  unsigned char source[6];        // AX25 Mode Source Call Sign(default NOCALL)
        #   20  unsigned char destination[6];   // AX25 Mode Destination Call Sign(default CQ)
        #   26  uint_2 tx_preamble;             // AX25 Mode Tx Preamble Byte Length(0x00 = 20 flags)
        #   28  uint_2 tx_postamble;            // AX25 Mode Tx Postamble Byte Length(0x00 = 20 flags)
        #   30  uint_2 function_config;         // Radio Configuration Discrete Behaviors
        #   32  uint_2 function_config2;        // Radio Configuration Discrete Behaviors  # 2
        #   } RADIO_CONFIGURATION_TYPE;

        self.packetHeader = bytearray(8)
        # Li Packet Header
        # Sync (2B), Command Type (2B), Payload Size (2B), Header Check SUm (2B) = 8 Bytes
        self.packetHeader[0] = 0x48
        self.packetHeader[1] = 0x65

    def helloWorld(self):
        print('Lithium: Hello World')

        #int.from_bytes(self.telemetryBytes[0:2], byteorder='big', signed=False) )
    def printTelemetry(self):
        print('Lithium Telemetry:')
        print('Op Counter: ', int.from_bytes(self.telemetryBytes[0:2], byteorder='big', signed=False) )
        print('uProc Temp: ', int.from_bytes(self.telemetryBytes[2:4], byteorder='big', signed=True) )
        print('Time Count: ', int.from_bytes(self.telemetryBytes[4:7], byteorder='big', signed=True) )
        print('RSSI:       ', self.telemetryBytes[7] )
        print('Bytes rxd:  ', int.from_bytes(self.telemetryBytes[8:12], byteorder='big', signed=True) )
        print('Bytes txd:  ', int.from_bytes(self.telemetryBytes[12:16], byteorder='big', signed=True) )
        return

    def printConfiguration(self):
        print('Lithium Configuration:')
        print('\tIF bps:    ', self.radioConfiguration[0], ' ', self.IF_BAUD[self.radioConfiguration[0]])
        print('\tTX PA lvl: ', self.radioConfiguration[1] )
        print('\tTX baud:   ', self.radioConfiguration[3], ' ', self.RF_BAUD[self.radioConfiguration[3]])
        print('\tTX mod:    ', self.radioConfiguration[5], ' ', self.MODULATION[self.radioConfiguration[5]])
        print('\tTX freq:   ', int.from_bytes(self.radioConfiguration[10:14], byteorder='big', signed=False) )
        print('\tRX baud:   ', self.radioConfiguration[2], ' ', self.RF_BAUD[self.radioConfiguration[2]])
        print('\tRX mod:    ', self.radioConfiguration[4], ' ', self.MODULATION[self.radioConfiguration[4]])
        print('\tRX freq:   ', int.from_bytes(self.radioConfiguration[6:10], byteorder='big', signed=False) )
        print('\tSrc call:  ', "".join(map(chr,self.radioConfiguration[14:20])))
        print('\tDst call:  ', "".join(map(chr,self.radioConfiguration[20:26])))
        print('\tTX preAm:  ',int.from_bytes(self.radioConfiguration[26:28], byteorder='big', signed=False) )
        print('\tTX postAm: ',int.from_bytes(self.radioConfiguration[28:39], byteorder='big', signed=False) )
        print('\tFunc 1:     ', end='')
        print (' '.join('0x{:02x}'.format(x) for x in self.radioConfiguration[30:32]))
        print('\tFunc 2:     ', end='')
        print (' '.join('0x{:02x}'.format(x) for x in self.radioConfiguration[32:34]))
        #int.from_bytes(self.radioConfiguration[6:10], byteorder='big', signed=False)

    def setSrcCall(self, src):
        l = len(src)
        if l > 6:
            l = 6
        b = bytearray()
        b.extend(map(ord,src))
        for i in range(0,l):
            self.radioConfiguration[14+i] = b[i]
        return

    def setDstCall(self, src):
        l = len(src)
        if l > 6:
            l = 6
        b = bytearray()
        b.extend(map(ord,src))
        for i in range(0,l):
            self.radioConfiguration[20+i] = b[i]
        return


    def setTxFreq(self, freq):
        bytes = intToByteArray( freq, 4 )
        print('Len bytes:', len(bytes))
        #self.radioConfiguration[10:13] = bytes # For ssome reason this was adding a byte extra?
        self.radioConfiguration[10] = bytes[0]
        self.radioConfiguration[11] = bytes[1]
        self.radioConfiguration[12] = bytes[2]
        self.radioConfiguration[13] = bytes[3]
        return

    def setRxFreq(self, freq):
        bytes = intToByteArray( freq, 4 )
        print('Len bytes:', len(bytes))
        #self.radioConfiguration[6:9] = bytes # For ssome reason this was adding a byte extra?
        self.radioConfiguration[6] = bytes[0]
        self.radioConfiguration[7] = bytes[1]
        self.radioConfiguration[8] = bytes[2]
        self.radioConfiguration[9] = bytes[3]
        return


class pyLiShell(cmd.Cmd):
    intro = 'Welcome to the pyLiShell.'
    prompt = 'pyLi>'
    #liD = None

    HOST = 'localhost'
    PORT = 12601
    # FILE = 'temp.txt'
    # WRAP_NUM = 10

    def __init__(self):
        #global liD
        print('Testing init.')
        self.liD = lithium()
        cmd.Cmd.__init__(self)


    #--------------------------------------------------------------------------
    # Shell commands
    #--------------------------------------------------------------------------
    def do_exit(self, arg):
        'Exit'
        sys.exit()

    def do_connect(self, arg):
        'Connect to serial port: connect PORT BAUD_RATE'
        self.connectSerialPort(arg)

    def do_noop(self, arg):
        'Send a no_op (no operation)'
        self.sendCommand(0x1001, None )
        self.receiveAck()

    def do_sockrx(self, arg):
        'Received data from radio and send to socket'
        self.rxDataFromRadio()
        return

    def do_socktx(self, arg):
        'Send data to radio from socket'
        self.txDataFromSocket()
        return

    def do_setsrc(self, arg):
        'Set SRC call sign'
        self.liD.setSrcCall( arg )
        self.sendCommand( 0x0106, self.liD.radioConfiguration )
        self.receiveAck()

    def do_setdst(self, arg):
        'Set DST call sign'
        self.liD.setDstCall( arg )
        self.sendCommand( 0x0106, self.liD.radioConfiguration )
        self.receiveAck()

    def do_qc(self, arg):
        'Query configuation'
        self.sendCommand(0x1005, None)
        self.receiveConfiguration()
        printBytes( self.liD.radioConfiguration )
        self.liD.printConfiguration()

    def do_qt(self, arg):
        'Query telemetry '
        self.sendCommand(0x1007, None)
        self.receiveTelemetry()
        #printBytes( self.liD.radioTelemetry )
        self.printTelemetry()

    def do_settx(self, arg):
        'Set TX frequency'
        if (len(arg) != 6) or ( arg.isdigit() == False ):                   # Check Length and that we have a number
            print('ERROR: Argument must be a six digit number, for example, 437100.')
            return
        self.liD.setTxFreq( int(arg) )
        self.sendCommand( 0x0106, self.liD.radioConfiguration )
        self.receiveAck()


    def do_setrx(self, arg):
        'Set RX frequency'
        if (len(arg) != 6) or ( arg.isdigit() == False ):                   # Check Length and that we have a number
            print('ERROR: Argument must be a six digit number, for example, 437100.')
            return
        self.liD.setRxFreq( int(arg) )
        self.sendCommand( 0x0106, self.liD.radioConfiguration )
        self.receiveAck()


    def do_test(self, arg):
        'test code'
        print('Hello world.')
        print('-->  ', )
        self.liD.helloWorld()

    #--------------------------------------------------------------------------
    # Other functions
    #--------------------------------------------------------------------------

    def transmitData(self,data):
        self.sendCommand( 0x0103, data )
        self.receiveAck()
        return

    def receiveData(self,data):
        print('Receiving data.')
        bytes = readSerial()
        l = len(bytes)
        # Check checksums
        if self.checkCheckSum(bytes[2:l]) == False:
            print('ERROR: failed checksum')

        return( bytes[8:l-2] )

    def txDataFromSocket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((self.HOST, int(PORT)))
        except socket.error as msg:
            print('\tBind failed.  Error Code: ', str(msg[0]), ' Message ', msg[1])
            break

        try:
            while True:
                data = s.recv(255)
                self.transmitData(data)
        finally:
            s.close()

        return

    def rxDataFromRadio(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((self.HOST, int(PORT)))
        except socket.error as msg:
            print('\tBind failed.  Error Code: ', str(msg[0]), ' Message ', msg[1])
            break

        try:
            while True:
                # rx data from serial
                data = receiveData()
                # send to socket
                s.sendall( data )
        finally:
            s.close()

        return

    def checkCheckSum(self, bytes):
        l = len(bytes)
        chkA, chkB = self.calculate_checksum( bytes, l-2)   # Set Header Checksums
        if chkA != bytes[len-2]:
            return False
        if chkB != bytes[len-1]:
            return False
        return True

    def receiveConfiguration(self):
        print('Receiving configuration.')
        # Read in from serial, check for number of bytes
        bytes = readSerial()
        l = len(bytes)
        # Check checksums
        if l != (8+34+2):
            print('ERROR: invalid # of bytes(',l,')')
            return
        if self.checkCheckSum(bytes[2:l]) == False:
            print('ERROR: failed checksum')
        # Extract Configuration
        self.liD.radioConfiguration = bytes[8:l-2]
        return

    def readSerial(self):
        bytes = bytearray()
        while self.serialPort.inWaiting() > 0:
            bytes.append(self.serialPort.read())
        return( bytes )

    def receiveAck(self):
        print('Receiving ack.')
        bytes = readSerial()
        if len(bytes) != 8:
            print('ERROR: invalid # of bytes(',len(bytes),')')
            return
        if checkCheckSum(bytes(2:)) == False:
            print('ERROR: failed checksum')
            return

        # Parse for ack
        if (bytes[4] == 0x0A) && (bytes[5] == 0x0A):
            print ('ACK received.')
        else:
            print ('NACK received.')
        return


    def receiveTelemetry(self):
        print('Receiving telemetry.')

        # Read in from serial, check for number of bytes
        bytes = readSerial()
        l = len(bytes)
        # Check checksums
        if l != (8+16+2):
            print('ERROR: invalid # of bytes(',l,')')
            return
        if self.checkCheckSum(bytes[2:l]) == False:
            print('ERROR: failed checksum')
        # Extract Telemetry
        self.liD.radioConfiguration = bytes[8:l-2]
        return


    def printConfiguration(self):
        self.liD.print('Printing configuration.')

    def printTelemetry(self):
        print('Printing telemetry.')
        self.liD.printTelemetry()

    def getByte(self, n, value):
        # Shift n times
        offset = n*8
        return value>>offset & 0xFF

    def calculate_checksum( self, data, count):
        sum1 = 0
        sum2 = 0
        for i in range(0,count):
            sum1 = (sum1+data[i]) % 255
            sum2 = (sum2+sum1) % 255
        return sum2, sum1

    def sendCommand(self, cmdID, args):
        #print('Sending command: ', cmdID)

        headerLen = 8
        cmdLen = 8
        cmd = bytearray(cmdLen)
        cmd[0] = 0x48               # set sync bytes
        cmd[1] = 0x65
        cmd[2] = self.getByte(1, cmdID)  # set command type
        cmd[3] = self.getByte(0, cmdID)

        if args:            # set payload size
            argLen = len(args)
            if argLen > 256:
                print('Error: command arguments too long.')
                return
            cmdLen += argLen+2              # Add arg len plus 2 bytes for the payload header
            cmd[4] = self.getByte(1, argLen)
            cmd[5] = self.getByte(0, argLen)
            cmd[6],cmd[7] = self.calculate_checksum( cmd, 6)   # Set Header Checksums
            #cmd.resize(cmdLen)
            #for i in range(0,argLen):
            #    cmd[headerLen+i] = args[i]
            cmd[headerLen:] = args          # Appendar args
            cmd[cmdLen-2:] = self.calculate_checksum( cmd[2:cmdLen-2], (cmdLen-4)) # Append checksum
        else:
            cmd[4] = 0
            cmd[5] = 0
            cmd[6],cmd[7] = self.calculate_checksum( cmd, 6)   # Set Header Checksums
            #argLen = 0

        printBytes( cmd )

        sendToSerial( cmd )
        return

    def sendToSerial(self, data):
        self.serialPort.write(data.encode())


    # ----------------------------------
    def connectSerialPort(self, arg):
    # ----------------------------------
        # l = arg.split()
        # if len(l) != 2:
        #    print ("Invalid number of arguments: connect port baud_rate")
        #    return

        # FR: Add error checking.
        # p = l[0]
        # b = l[1]
        p = 'COM3'
        b = 9600

        self.serialPort = serial.Serial(
            port=p,
            baudrate=b,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )

        try:
            self.serialPort.isOpen()
            print('Success: serial port opened.')
        except:
            print('Failed: serial port not opened.')
            return


    # extra
    def sendFileToTCP(self, port):
        # open file
        # open socket
        # write file to socket
        # close file
        # close socket

        if (port == ''):
            port = 12601

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((self.HOST, int(port)))
        except s.error as msg:
            print('\tBind failed.  Error Code: ', str(msg[0]), ' Message ', msg[1])
            return

        fileName = ''
        if (fileName == ''):
            fileName = "echo_client.py"
        with open(fileName, "rb") as f:
            byte = f.read(1)
            while byte:
                s.sendall(byte)
                byte = f.read(1)
        return
        f.close()
        s.close()
        print('\tFile sent')


    # extra
    def sendConsoleToTCP(self, port):
        if (port == ''):
            port = 12601

        data = ""

        while (data != 'x'):
            data = input('Enter data:')
            if (data != 'x'):
                print('\tSending: ', data)

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((self.HOST, int(port)))
            except socket.error as msg:
                print('\tBind failed.  Error Code: ', str(msg[0]), ' Message ', msg[1])
                break
            s.sendall(data.encode())
            s.close()

        print('\tall done')
        return


    # extra
    def printHex(self, fileName):
        i = 1
        # target = open(self.FILE,'rb')
        if (fileName == ''):
            fileName = self.FILE
        with open(self.FILE, "rb") as f:
            byte = f.read(1)
            while byte:
                # print(byte.decode("utf-8"),end=" ", flush=True)
                print(format(int.from_bytes(byte, byteorder='big'), '02x'), end=" ")
                if (i == self.WRAP_NUM):
                    print("")
                    i = 0
                byte = f.read(1)
                i = i + 1
            print("")
        return


    # extra
    def rcvDataWriteFile(self, port):
        # Open a server socket

        if (port == ''):
            port = 12601

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("\tSocket created: ", port)

        try:
            s.bind((self.HOST, int(port)))
        except socket.error as msg:
            print('\tBind failed.  Error Code: ', str(msg[0]), ' Message ', msg[1])
            sys.exit()

        print("\tSocket bind complete.")
        s.listen(10)
        print('\tSocket now listening')
        conn, addr = s.accept()
        print('\tConnected with ', addr[0], ':', str(addr[1]))

        target = open(self.FILE, 'ab+')
        data = conn.recv(1024)
        while (len(data) != 0):
            print('\tRcvd (', len(data), '):', data.decode(encoding='UTF-8'))
            print('\tRcvd (', len(data), '):')
            if not data:
                print('\tData empty.')
            else:
                print('\tWriting data.')
                da = bytearray(data)
                target.write(data)
            data = conn.recv(1024)
        target.close()
        s.close()


if __name__ == '__main__':
    #pyLiShell().cmdloop()
    pyLi = pyLiShell()
    pyLi.do_settx("437001")
    pyLi.do_setrx("437002")
    pyLi.do_setdst("CQ")
    pyLi.do_setsrc("KF6RFX")
    pyLi.do_qc(None)
    pyLi.do_qt(None)
