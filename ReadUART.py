"""
This class establishes serial connection with the radar, send configuration and receives data.
"""

import serial
import time
import numpy as np
from datetime import datetime
from threading import Thread


class UartData():
    def __init__(self,configFileName,comport = 'COM8',dataport = 'COM6') -> None:
        
        # ports and baudrates for COM and Data
        self.comport = comport
        self.dataport = dataport
        self.combaud = 115200
        self.datarate = 921600
        self.vital_start=0
        # configuration file path
        self.configFileName = configFileName
        #self.serialConfig()
        # parse the config file
        self.cnfigParameters = self.parseConfigFile()
        self.byteBuffer = np.zeros(2**16,dtype = 'uint8')
        self.byteBufferLength = 0
        self.record = False
        self.stop=False
        self.RangeAzheatmapRx1 = np.zeros((256, 400), dtype=np.float32)
        self.RangeAzheatmapRx2 = np.zeros((256, 400), dtype=np.float32)
        self.r = 0
        self.RX1_alt = []
        self.RX2_alt = []
        self.timewindow = 0
        self.timeshift = 100
        self.Freq_BR = []
        self.Freq_HR= []
        self.RR = []
        self.HR = []
        self.measure = 0
        self.shpBR = 0
        self.shpHR = 0
        self.sharpresultBR = []
        self.sharpresultHR = []
        self.FltAvg = 3
        self.newinputpeakbr = 0
        self.tempbr = 0
        self.newinputpeakhr = 0
        self.temphr = 0
        self.FS=20
        self.time=np.arange(0,20,(1/self.FS))
        # Function to configure the serial ports and send the data from
    # the configuration file to the radar
    def serialConfig(self):
    
        # Open the serial ports for the configuration and the data ports
        
        # Raspberry pi
        #CLIport = serial.Serial('/dev/ttyACM0', 115200)
        #Dataport = serial.Serial('/dev/ttyACM1', 921600)
            
        # Windows
        #self.CLIport = serial.Serial(self.comport, self.combaud)
        #self.Dataport = serial.Serial(self.dataport, self.datarate)
        # Windows
        self.CLIport = serial.Serial(self.comport, self.combaud)
        self.Dataport = serial.Serial(self.dataport, self.datarate)
        # Read the configuration file and send it to the board
        config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        for i in config:
            self.CLIport.write((i+'\n').encode())
            print(i)
            # time.sleep(0.01)
            #print(self.CLIport.readline())
            #print(self.CLIport.readline())
            time.sleep(0.1)
            
        return None
    

    # ------------------------------------------------------------------  

    # Function to parse the data inside the configuration file
    def parseConfigFile(self):
        self.configParameters = {} # Initialize an empty dictionary to store the configuration parameters
        
        # Read the configuration file and send it to the board
        config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        for i in config:
            
            # Split the line
            splitWords = i.split(" ")
            
            # Hard code the number of antennas, change if other configuration is used
            # numRxAnt = 4
            # numTxAnt = 3
            
            if "channelCfg" in splitWords[0]:
                # numRxAnt = bin(splitWords[1])
                # numRxAnt = (len(str(bin(splitWords[1]))))-2
                
                # numRxAnt = (len(bin(splitWords[1])))-2
                # numTxAnt = (len(bin(splitWords[2])))-2
                
                numRxAnt = (len(bin(int(splitWords[1]))))-2
                numTxAnt = (len(bin(int(splitWords[2]))))-2
            
            # Get the information about the profile configuration
            if "profileCfg" in splitWords[0]:
                startFreq = int(float(splitWords[2]))
                idleTime = int(splitWords[3])
                rampEndTime = float(splitWords[5])
                freqSlopeConst = float(splitWords[8])
                numAdcSamples = int(splitWords[10])
                numAdcSamplesRoundTo2 = 1
                
                while numAdcSamples > numAdcSamplesRoundTo2:
                    numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                    
                digOutSampleRate = int(splitWords[11])
                
            # Get the information about the frame configuration    
            elif "frameCfg" in splitWords[0]:
                
                chirpStartIdx = int(splitWords[1])
                chirpEndIdx = int(splitWords[2])
                numLoops = int(splitWords[3])
                numFrames = int(splitWords[4])
                framePeriodicity = float(splitWords[5])

                
        numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
        self.configParameters["numDopplerBins"] = numChirpsPerFrame // numTxAnt
        self.configParameters["numRangeBins"] = numAdcSamplesRoundTo2
        self.configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
        self.configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * self.configParameters["numRangeBins"])
        self.configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * self.configParameters["numDopplerBins"] * numTxAnt)
        self.configParameters["chirpDuration_us"] = (1e3*numAdcSamples)/(digOutSampleRate)
        freqSlopeConst_temp = 48*freqSlopeConst* 2**26 * 1e3/((3.6*1e9)*900);  # To match the C-code 
        
        self.configParameters["chirpBandwidth_kHz"] = (freqSlopeConst_temp)*(self.configParameters["chirpDuration_us"])
        numTemp = (self.configParameters["chirpDuration_us"])*(digOutSampleRate)*(3e8)
        denTemp = 2*(self.configParameters["chirpBandwidth_kHz"])
        # self.configParameters["rangeMaximum"] =  numTemp/(denTemp*1e9)
        self.configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
        self.configParameters["rangeMaximum"] = self.configParameters["maxRange"]
        self.configParameters["rangeBinSize_meter"] = self.configParameters["rangeMaximum"]/(self.configParameters["numRangeBins"])
        
        rangeStartMeters = 0
        rangeEndMeters = self.configParameters["rangeMaximum"]
        
        rangeStart_Index = int(rangeStartMeters/self.configParameters["rangeBinSize_meter"])
        rangeEnd_Index   = int(rangeEndMeters/self.configParameters["rangeBinSize_meter"])
        self.configParameters["lambdaCenter_mm"] = (3e8/(startFreq))/1e6
        self.configParameters["rangeStart_Index"] = rangeStart_Index
        self.configParameters["rangeEnd_Index"] = rangeEnd_Index
        self.configParameters["numRangeBinProcessed"] = rangeEnd_Index - rangeStart_Index + 1
   
        return self.configParameters

    # Funtion to read and parse the incoming data
    def readAndParseData18xx(self, q):
        
        # Constants
        OBJ_STRUCT_SIZE_BYTES = 12
        BYTE_VEC_ACC_MAX_SIZE = 2**15
        MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        MMWDEMO_UART_MSG_RANGE_PROFILE   = 2
        MMWDEMO_OUTPUT_MSG_NOISE_PROFILE = 3
        MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP = 4
        MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
        MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST = 10
        maxBufferSize = 2**16
        tlvHeaderLengthInBytes = 8
        pointLengthInBytes = 16
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
        
        # Initialize variables
        magicOK = 0 # Checks if magic number has been read
        dataOK = 0 # Checks if the data has been read correctly
        frameNumber = 0
        detObj = {}
        rangeDoppler = 0
        rangeArray = {}
        dopplerArray = {}
        rangeDoppler = {}
        data = 3*np.ones(128)
        
        readBuffer = self.Dataport.read(self.Dataport.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
        byteCount = len(byteVec)
        
        # Check that the buffer is not full, and then add the data to the buffer
        if (self.byteBufferLength + byteCount) < maxBufferSize:
            self.byteBuffer[self.byteBufferLength:self.byteBufferLength + byteCount] = byteVec[:byteCount]
            self.byteBufferLength = self.byteBufferLength + byteCount
            
        # Check that the buffer has some data
        if self.byteBufferLength > 16:
            
            # Check for all possible locations of the magic word
            possibleLocs = np.where(self.byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = self.byteBuffer[loc:loc+8]
                if np.all(check == magicWord):
                    startIdx.append(loc)
                
            # Check that startIdx is not empty
            if startIdx:
                
                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < self.byteBufferLength:
                    self.byteBuffer[:self.byteBufferLength-startIdx[0]] = self.byteBuffer[startIdx[0]:self.byteBufferLength]
                    self.byteBuffer[self.byteBufferLength-startIdx[0]:] = np.zeros(len(self.byteBuffer[self.byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                    self.byteBufferLength = self.byteBufferLength - startIdx[0]
                    
                # Check that there have no errors with the byte buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0
                    
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]
                
                # Read the total packet length
                totalPacketLen = np.matmul(self.byteBuffer[12:12+4],word)

                # Check that all the packet has been read
                if (self.byteBufferLength >= totalPacketLen) and (self.byteBufferLength != 0):
                    magicOK = 1

        # If magicOK is equal to 1 then process the message
        if magicOK:
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Initialize the pointer index
            idX = 0
            
            # Read the header
            magicNumber = self.byteBuffer[idX:idX+8]
            idX += 8
            version = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            totalPacketLen = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            platform = format(np.matmul(self.byteBuffer[idX:idX+4],word),'x')
            idX += 4
            frameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            timeCpuCycles = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numDetectedObj = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numTLVs = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            subFrameNumber = np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            numStaticDetectedObj=np.matmul(self.byteBuffer[idX:idX+4],word)
            idX += 4
            # Read the TLV messages
            for tlvIdx in range(numTLVs):

                datapayload=0
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2**8, 2**16, 2**24]

                # Check the header of the TLV message
                tlv_type = np.matmul(self.byteBuffer[idX:idX+4],word)
                #print(tlv_type)
                # print('frame number: ', frameNumber)
                idX += 4
                tlv_length = np.matmul(self.byteBuffer[idX:idX+4],word)
                idX += 4
                datapayload=self.byteBuffer[idX:idX+tlv_length]
                idX += tlv_length
                if tlv_type ==MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:    
                    # 128 = number of FFT doppler bins in the C code. it is at minimum the next power of 2 of num Doppler chirps
                    numBytes = 2*128
                    #payload = self.byteBuffer[idX:idX + numBytes]
                    #idX += numBytes
                    #data = payload.view(dtype=np.int16)
                    #print(noiseProfile)
                    #q.put(data)
                    # save doppler vector to local file
                    out = './doppler/' + str(time.time()) + '.txt'
                    #print('time:', datetime.now().strftime('%H:%M:%S.%f'))
                    #np.savetxt(out, data)
                    print(None)
                elif tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                      print('DetObj')

                elif tlv_type == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP:
                    #print("Static Heat Map")
                    #q1 = np.zeros(np.size(datapayload), dtype=np.int16)
                    numVirtualAntenna = 4 * 3
                    #print("Payload size {0}\nData: {1}".format(len(datapayload), datapayload))
                    q1 = datapayload[0::2]
                    q2 = datapayload[1::2]

                    q3 = []
                    for k in range(len(q1)):
                        q3.append(q1[k] + 256 * q2[k])

                    #q1[q1 > 32767] = q1[q1 > 32767] - 65536
                    for k in range(len(q3)):
                        if q3[k] > 32767:
                            q3[k] -= 65536
                    qImg = q3[0::2]
                    qReal = q3[1::2]
                    qComplex = []
                    for k in range(len(qReal)):
                        qComplex.append(1j * qImg[k] + qReal[k]) 
                   
                    #q1 = 1j * q1[0::2] + q1[1::2]

                    #q1RX1 = np.reshape(q1[0:256], (256, 1))
                    #q1RX2 = np.reshape(q1[256:], (256, 1))
                    self.timewindow += 1


                    if self.timewindow < 401:

                        self.RX1_alt.extend([qComplex[0:256]])
                        self.RX2_alt.extend([qComplex[256:]])

                    else:
                        def get_row(row_index, arr):
                            retval = []
                            for i in range(len(arr)):
                                retval.append(arr[i][row_index])
                            return retval

                        def get_difference(row_A, row_B):
                            difval = []
                            for i in range(len(row_A)):
                                difval.append(abs(row_A[i]) - abs(row_B[i]))
                            diff_sum = 0
                            #print("Diffvalues: {0}\n".format(difval))
                            for i in range(len(difval)):
                                diff_sum = diff_sum + difval[i]
                            return diff_sum
                        distanceTarget = 0.5
                        ResRange=self.configParameters["maxRange"] /self.configParameters["numRangeBins"]
                        preindex = int(distanceTarget / ResRange)
                        temp1 = get_row(preindex - 10, self.RX1_alt)
                        time.sleep(0.01)
                        indexrowtarget = 0
                        for i in range(-9, 11):
                            selectedrow = preindex + i
                            #print(selectedrow)
                            diff1 = 0
                            diff1 = get_difference(get_row(selectedrow, self.RX1_alt), temp1)
                            if diff1 > 0:
                                temp1 = get_row(selectedrow, self.RX1_alt)
                                indexrowtarget = selectedrow
                        row_targetRX1 = []
                        row_targetRX2 = []

                        for k in range(len(self.RX1_alt)):
                            row_targetRX1.append(self.RX1_alt[k][indexrowtarget])
                            row_targetRX2.append(self.RX2_alt[k][indexrowtarget])
                        np.savetxt("recordings/range_profile_"+str(self.measure),row_targetRX1)
                        print(self.measure)
                        self.measure += 1
                        del self.RX1_alt[0:(self.timeshift)]
                        del self.RX2_alt[0:(self.timeshift)]
                        self.timewindow = self.timewindow - (self.timeshift + 1)
                        
            # Remove already processed data
            if idX > 0 and self.byteBufferLength>idX:
                shiftSize = totalPacketLen
                self.byteBuffer[:self.byteBufferLength - shiftSize] = self.byteBuffer[shiftSize:self.byteBufferLength]
                self.byteBuffer[self.byteBufferLength - shiftSize:] = np.zeros(len(self.byteBuffer[self.byteBufferLength - shiftSize:]),dtype = 'uint8')
                self.byteBufferLength = self.byteBufferLength - shiftSize

                # Check that there are no errors with the buffer length
                if self.byteBufferLength < 0:
                    self.byteBufferLength = 0

        yield data

    def serial_loop(self, q,run_event):
        """read UART data

        Args:
            q (_type_): queue to put received data and send to other threads
        """
        while run_event.is_set():

            next(self.readAndParseData18xx(q))

            #if self.vital_start == 1:
                #run_event.clear()
                #configFileName
                #uartdata = UartData(configFileName=configFileName, comport='COM4', dataport='COM5')
                #uartdata.serialConfig()
                #run_event.set()
                #self.vital_start=0

        self.CLIport.write(('sensorStop\n').encode())
        print('>>>',self.CLIport.readline())
        #time.sleep(0.5)
        self.CLIport.write(('sensorReset\n').encode())
        print('>>>',self.CLIport.readline())
        self.CLIport.close()
        print('>>>','sensor stopped')
        self.Dataport.close()
        print('>>>','ports closed')

# Step 1: Serial connection Thread
class Worker_serial(Thread):
    def __init__(self, configFileName, comport, dataport,run_event,q):
        """ Thread class to receive UART data

        Args:
            configFileName (string): pathto the configuiration file
            comport (string): port number
            dataport (string): data port number
        """
        Thread.__init__(self)
        self.configFileName = configFileName
        self.comport = comport
        self.dataport = dataport
        self.run_event = run_event
        self.q = q

    def run(self):
        """
        send configuration and read UART data, the received data is sent through a queue to other threads for further processing.
        """
        self.uartdata = UartData(configFileName=self.configFileName,comport = self.comport,dataport = self.dataport)
        self.uartdata.serialConfig()
        self.uartdata.serial_loop(self.q,self.run_event)


# -------------------------    MAIN   -----------------------------------------
if __name__ == "__main__":
    import queue, threading
    # warm reset sensor
    # sensorStopReset()
    # Configurate the serial port
    configFileName = r"Configuration_workwithAreaScanner.cfg"
    uartdata = UartData(configFileName=configFileName,comport = 'COM8',dataport = 'COM6')
    uartdata.serialConfig()
    # Main loop to get data coming from Uart
    q = queue.Queue()
    run_event = threading.Event()
    run_event.set()
    uartdata.serial_loop(q,run_event)