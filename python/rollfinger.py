#from https://github.com/pyserial/pyserial/blob/master/serial/rfc2217.py
#from rfc2217 import Serial
import time
import numpy as np
from serial import Serial

#ser=Serial('rfc2217://192.168.255.10:2217', 115200)

#for i in range(10):
#    print(ser.readline())
#    i=i+1
#    #time.sleep(0.01)
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class RollFinger():
    def __init__(self, port, baudrate):
        self.ser = Serial(port, baudrate, timeout =0.01)
        #time.sleep(0.5)
        #self.ser.write(str.encode('R'))
        time.sleep(0.5)
        self.fPosThreshold = 10
        self.proxThreshold =10
        #self.prev_data=np.zeros(25)
        self.prev_data = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.rdr = ReadLine(self.ser)
    def flushBuffer(self):
        #time.sleep(0.15)
        #self.ser.write(str.encode('R'))
        #time.sleep(0.15)
        pass

    def setLFingerPos(self,posval):
        self.ser.write(str.encode('clp'+format(posval,'03d')+'e'))
        #if(posval>100):
        #    self.ser.write(b'l')
        #else:
        #    self.ser.write(b'k')
        time.sleep(0.05)
    def setRFingerPos(self,posval):
        self.ser.write(str.encode('crp'+format(posval,'03d')+'e'))
        #if(posval>100):
        #    self.ser.write(b'r')
        #else:
        #    self.ser.write(b'e')
        
        time.sleep(0.05)    
    def openGripper(self):
        self.ser.write(str.encode('cop000e'))
        #self.ser.write(b'o')
        time.sleep(0.05)
    def closeGripper(self, current=0):
        self.ser.write(str.encode('ccp'+format(current,'03d')+'e'))
        #self.ser.write(b'c')
        time.sleep(0.05)
    def readSerial(self):
        raw_data = self.ser.readline()
        #raw_data=self.rdr.readline()
        data_ar = raw_data.split(b'\t')
        #check for start - stop bits and length of data array
        try:
            if((chr(raw_data[0])=='s') and(chr(raw_data[-2])=='e') and (len(data_ar)==28)):
                data_ar = data_ar[1:-1]
                #print(data_ar)
                try:
                    data = np.array([int(x) for x in data_ar])
                    self.prev_data = data
                    return data
                except:
                    return self.prev_data
            
            else:
                #return False
                return self.prev_data
        except:
            return self.prev_data        
    def getGripperCurrent(self):
        data= self.readSerial()
        return data[0]
    def getGripperPositions(self):
        data= self.readSerial()
        return data[1:3]
    def getFingerPositions(self):
        data= self.readSerial()
        return data[3:5]
    def getProxData(self):
        data= self.readSerial()
        data[data<0 ] = 0
        return data[5:-1]
    def getProxTip(self):
        data = self.getProxData()
        data=np.array([ data[11], data[10], data[1],data[2]])
        data[data<self.proxThreshold ] = 0
        return data
    def getProxInFinger(self):
        data = self.getProxData()
        data=np.array([data[9], data[8], data[7]])
        data[data<self.proxThreshold ] = 0
        return data
    def getProxSides(self):
        data = self.getProxData()
        data=np.array([data[14],data[16],data[15],data[12],data[13],data[6],data[5],data[4],data[3],data[0]]) 
        data[data<self.proxThreshold ] = 0
        return data
    def getGripperGap(self):
        data = self.readSerial()
        return data[-1]
    def setGapZero(self):
        self.ser.write(str.encode('cgz000e'))
        for i in range(100):
            self.ser.readline()
    def setGripperGap(self, posval):
        self.ser.write(str.encode('cgg'+format(posval,'03d')+'e'))



        
    def moveRollP1(self):
        self.setLFingerPos(100)
        self.setRFingerPos(100)
        self.setLFingerPos(100)
        self.setRFingerPos(100)
        time.sleep(0.5)
        FP =self.getFingerPositions()
        LP = FP[1]
        RP = FP[0]
        t0 = time.time()
        while((abs(FP[0]-100)>self.fPosThreshold)or(abs(FP[1]-100)>self.fPosThreshold)):
            FP =self.getFingerPositions()
            #print("moving ", FP) 
        time.sleep(0.5)
        print("move P1 over ", time.time()-t0)

    def moveRollP2(self):
        self.setLFingerPos(0)
        self.setRFingerPos(200)
        self.setLFingerPos(0)
        self.setRFingerPos(200)
        time.sleep(0.5)
        
        FP =self.getFingerPositions()
        t0 = time.time()
        #while((abs(FP[0]-200)>self.fPosThreshold)or(abs(FP[1]-0)>self.fPosThreshold)):
        #    FP =self.getFingerPositions()
        time.sleep(0.6)
        print("move P2 over ", time.time()-t0)
    def moveRollP3(self):
        self.setLFingerPos(200)
        self.setRFingerPos(0)
        self.setLFingerPos(200)
        self.setRFingerPos(0)
        time.sleep(0.5)
        FP =self.getFingerPositions()
        t0 = time.time()
        #while((abs(FP[0]-0)>self.fPosThreshold)or(abs(FP[1]-200)>self.fPosThreshold)):
        #    FP =self.getFingerPositions()
            #TODO  -  save photos
            #TODO  -  speed  control
        time.sleep(1.5)
        print("move P3 over ", time.time()-t0)
            
    def moveRollP4(self):
        self.setLFingerPos(0)
        self.setRFingerPos(0)
        self.setLFingerPos(0)
        self.setRFingerPos(0)
        time.sleep(0.5)
        FP =self.getFingerPositions()
        t0 = time.time()
        #while((abs(FP[0]-0)>self.fPosThreshold)or(abs(FP[1]-0)>self.fPosThreshold)):
        #    FP =self.getFingerPositions()
        time.sleep(0.5)    
        print("move P4 over ", time.time()-t0)    
    def roll(self):
        self.moveRollP1()
        self.moveRollP2()
        #start image capture
        time.sleep(1)
        self.moveRollP3()
        #stop image capture
        time.sleep(1)
        self.moveRollP4()
    def close(self):
        self.ser.close()