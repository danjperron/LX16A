#!/usr/bin/python3
from time import sleep
from serial import Serial
import struct


class LX16A:

  SERVO_FRAME_HEADER        =0x55
  SERVO_MOVE_TIME_WRITE     =1
  SERVO_MOVE_TIME_READ      =2
  SERVO_MOVE_TIME_WAIT_WRITE=7
  SERVO_MOVE_TIME_WAIT_READ =8
  SERVO_MOVE_START          =11
  SERVO_MOVE_STOP           =12
  SERVO_ID_WRITE            =13
  SERVO_ID_READ             =14
  SERVO_ANGLE_OFFSET_ADJUST =17
  SERVO_ANGLE_OFFSET_WRITE  =18
  SERVO_ANGLE_OFFSET_READ   =19
  SERVO_ANGLE_LIMIT_WRITE   =20
  SERVO_ANGLE_LIMIT_READ    =21
  SERVO_VIN_LIMIT_WRITE     =22
  SERVO_VIN_LIMIT_READ      =23
  SERVO_TEMP_MAX_LIMIT_WRITE=24
  SERVO_TEMP_MAX_LIMIT_READ =25
  SERVO_TEMP_READ           =26
  SERVO_VIN_READ            =27
  SERVO_POS_READ            =28
  SERVO_OR_MOTOR_MODE_WRITE =29
  SERVO_OR_MOTOR_MODE_READ  =30
  SERVO_LOAD_OR_UNLOAD_WRITE=31
  SERVO_LOAD_OR_UNLOAD_READ =32
  SERVO_LED_CTRL_WRITE      =33
  SERVO_LED_CTRL_READ       =34
  SERVO_LED_ERROR_WRITE     =35
  SERVO_LED_ERROR_READ      =36


  # declaration de l'objet connection au port serie

  def __init__(self,Port="/dev/ttyUSB0",Baudrate=115200, Timeout= 0.001):
     self.serial = Serial(Port,baudrate=Baudrate,timeout=Timeout)
     self.serial.setDTR(1)
     self.TX_DELAY_TIME = 0.00002
     self.Header = struct.pack("<BB",0x55,0x55)


  # envoi du packet  ajout du header et du checksum
  def sendPacket(self,packet):
     sum = 0
     for item in packet:
        sum = sum + item
     fullPacket = bytearray(self.Header + packet + struct.pack("<B",(~sum) & 0xff))
     self.serial.write(fullPacket)

     sleep(self.TX_DELAY_TIME)

  # set mode moteur avec la vitesse   motorMode=1
  # sinon set mode servo   motorMode=0
  def motorOrServo(self,id,motorMode,MotorSpeed):
     packet = struct.pack("<BBBBBh",id,7,
                          self.SERVO_OR_MOTOR_MODE_WRITE,
                          motorMode,0,MotorSpeed)
     self.sendPacket(packet)

  # bouger le servo entre 0 et 1000 soit 0.24 degree resolution
  # rate est en ms  de 0(fast) a 30000(slow)
  def moveServo(self,id,position,rate=1000):
     packet = struct.pack("<BBBHH",id,7,
                          self.SERVO_MOVE_TIME_WRITE,
                          position,rate)
     self.sendPacket(packet)

  # change le ID du servo
  def setID(self,id,newid):
     packet = struct.pack("<BBBB",id,4,
                          self.SERVO_ID_WRITE,newid)
     self.sendPacket(packet)


if __name__ == '__main__':
   m1 = LX16A()
   m1.motorOrServo(1,1,200)
   sleep(2.0)
   m1.motorOrServo(1,0,0)
   sleep(0.1)
   m1.moveServo(1,0,2500)
   sleep(5)
   m1.moveServo(1,1000,2500)
