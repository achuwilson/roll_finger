import tkinter
#import tkMessageBox

import rollfinger
from rollfinger import RollFinger
gripper = RollFinger('/dev/ttyUSB0', 115200)
import time
top = tkinter.Tk()

for i in range(10):
    initGap  = gripper.getGripperGap()
click_count = 0

def helloCallBack2():
    print("callback2")
def helloCallBack():
    print("callback")
    #tkMessageBox.showinfo( "Hello Python", "Hello World")
def opengripper():
    print("opengripper")
    gripper.openGripper()

def closegripper():
    print("closegripper")
    gripper.closeGripper(28)

def getgap():
    global initGap
    for i in range(10):
        initGap  = gripper.getGripperGap()
    print("getgap ", gripper.getGripperGap())

def inc_gap():
    global click_count
    click_count = click_count + 1
    gapcmd  =  initGap+ click_count
    gripper.setGripperGap(gapcmd)
    print("gap++ ", gripper.getGripperGap())


def dec_gap():
    global click_count
    click_count =  click_count - 1
    gapcmd  =  initGap+ click_count
    gripper.setGripperGap(gapcmd)
    print("gap-- ", gripper.getGripperGap())

def finger_mid():
    gripper.setLFingerPos(100)
    gripper.setRFingerPos(100)

def roll():
    gripper.setLFingerPos(0)
    gripper.setRFingerPos(200)
    time.sleep(0.9)
    print("rev")
    gripper.setLFingerPos(200)
    gripper.setRFingerPos(0)
    time.sleep(1.25)
    finger_mid()

B0 = tkinter.Button(top, text ="Open",width=25, command = opengripper)
B1= tkinter.Button(top, text ="Close",width=25, command = closegripper)
B2= tkinter.Button(top, text ="getGap",width=25, command = getgap)
B3= tkinter.Button(top, text ="Gap+",width=25, command = inc_gap)
B4= tkinter.Button(top, text ="Gap-",width=25, command = dec_gap)
B5= tkinter.Button(top, text ="FingerMidPos",width=25, command = finger_mid)
B6= tkinter.Button(top, text ="Roll",width=25, command = roll)

B0.pack()
B1.pack()
B2.pack()
B3.pack()
B4.pack()
B5.pack()
B6.pack()

top.mainloop()

