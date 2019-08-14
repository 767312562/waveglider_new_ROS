#!/usr/bin/env python
'''thrust_control ROS Node'''
import rospy
from std_msgs.msg import Float64
from PID import PID
import time
import pigpio

class PID_controllor():
    
    def __init__(self):
        rospy.init_node('pwmbuilder', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        self.course_real=0.0
        self.course_desired=0.0
        rospy.Subscriber("/course_real", Float64, self.callback_real)
        rospy.Subscriber("/course_desired", Float64, self.callback_desired)
        controllor=PID(8, 0.5, 6, -10, 10, -10, 10)
        pi=pigpio.pi()
        pi.set_PWM_frequency(23,50)
        pi.set_PWM_range(23,20000)
        while not rospy.is_shutdown():
            F=controllor.update(self.course_real,self.course_desired)
            dc=-2.979e-07*F**6 + 1.496e-05*F**5 + 0.0003753*F**4 - 0.02246*F**3 -0.1342*F**2 + 21.09*F + 1497
            rospy.loginfo('the result is %f',dc)
            pi.set_PWM_dutycycle(23,dc)
            #time.sleep(5)
            rate.sleep()
  
    def callback_real(self, msg):

        rospy.loginfo("the real course now is : %f", msg.data)
        self.course_real=msg.data

    def callback_desired(self, msg):

        rospy.loginfo("the desired course now is : %f", msg.data)
        self.course_desired=msg.data


if __name__ == '__main__':
    try:
        PID_controllor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    pi=pigpio.pi()
    pi.set_PWM_frequency(23,50)
    pi.set_PWM_range(23,20000)
    pi.set_PWM_dutycycle(23,1500)
