#!/usr/bin/env python
# ROS
import rospy
from geometry_msgs.msg import Twist
import math
import time
from pyMultiwii import MultiWii

rc = [1500, 1500, 1500, 1000]
board = MultiWii('/dev/ttyUSB0')
rc_enable = False
angular = 0.0
control_roll = 0.0
control_pitch = 0.0
control_yaw = 0.0
z = 0
y = 0.0
class PID:
    
    def __init__(self, P=0.0, I=0.0, D=0.0, SP=0.0):
        
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.SetPoint = SP
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = time.time()
        
        self.clear()

    def clear(self):
                
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
                
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard
            
            
            if delta_time > 0.0:
                self.DTerm = delta_error / delta_time
                #print(self.DTerm)

            self.last_time = self.current_time      
            self.last_error = error # self.last_error = 5
            
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time        

def rc_callback(msg):
    global rc, control_roll, control_pitch, control_yaw  ,z,y
    z = msg.angular.z
    y = msg.linear.z
    rc = [msg.angular.x + control_roll, msg.angular.y + control_pitch, msg.angular.z + control_yaw, msg.linear.z]    
    #print rc    

def pid_control(msg):
    global control_roll, control_pitch
    control_roll = msg.linear.x
    control_pitch = msg.linear.y
    print(time.clock(), "   ", rospy.Time.now())

def main():
    global rc, rc_enable, angular,control_yaw
    rospy.init_node("arm_disarm_rc_pid")

    pid_sub = rospy.Subscriber('pid_xy', Twist, pid_control)
    rc_sub = rospy.Subscriber('multiwii_rc', Twist, rc_callback)            
    
    time.sleep(2)
    pid = PID(0, 0, 0, 120)
    while not rospy.is_shutdown():
        try:
            if z ==1000:
                rc_enable = False
                control_yaw = 0.0
                board.disarm()
            elif z ==2000:
                rc_enable = True    
                board.arm()                                         
            if rc_enable:
                pid.setKp(0.8)
                pid.setKi(0.01)                
                board.sendCMD(8, MultiWii.SET_RAW_RC, rc) 
                magx= board.getData(MultiWii.RAW_IMU)           
                mx = float(board.rawIMU['mx'])
                my = float(board.rawIMU['my'])
                angular = math.atan2(my,mx) *180 /math.pi
                pid.update(angular)                
                control_yaw = pid.output
                print(control_yaw)
                pid.ITerm = 0.0
        except:
            print("Oops. Something happened")
            break

    board.sendCMD(8, MultiWii.SET_RAW_RC, rc)
    board.disarm()
    


if __name__ == '__main__':
    main()
