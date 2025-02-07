#!/usr/bin/env python
'''path_following ROS Node'''
import rospy
from std_msgs.msg import UInt64MultiArray
from std_msgs.msg import Float64
from math import pi, acos
import numpy as np
from station_keeping import Station_keeping

class Path_following():

    def __init__(self):
          rospy.init_node('path_following', anonymous=True)
          rate = rospy.Rate(10) # 10hz
          self.pointsway=np.array(rospy.get_param('waypoints'))
          self.pub = rospy.Publisher('/course_desired', Float64, queue_size=10)
          self.radius=rospy.get_param('radius')
          self.deta=rospy.get_param('deta')

          rospy.Subscriber("/position_real", UInt64MultiArray, self.callback)
          rate.sleep()


    def p_f(self,realposition):
#该函数用来确认航迹段上的LOS点
#判断当前航迹段

        d=np.zeros((1, len(self.pointsway)))  
        for i in range(len(self.pointsway)):
            d[i]=np.norm(self.pointsway[i]-realposition)
        d=list(d)
        b,a = min(enumerate(d), key = operator.itemgetter(1)) #寻找最近轨迹点的位置
       
        if b==0:

            A=self.pointsway(b+1,1)-self.pointsway(b,1)
            B=self.pointsway(b,0)-self.pointsway(b+1,0)
            C=self.pointsway(b+1,0)*self.pointsway(b,1)-self.pointsway(b,0)*self.pointsway(b+1,1)
            #求垂足
            y0=(B*B*realposition[0,1]-A*B*realposition[0,0]-A*C)/(A*A+B*B)
            x0=(-A*B*realposition[0,1]+A*A*realposition[0,0]-B*C)/(A*A+B*B)
            yd=y0-np.sign(pointsway(b,0)-pointsway(b+1,0))*np.sqrt(np.square(self.deta)/np.square(1+(abs(pointsway(b,1)-pointsway(b+1,1))/\
            (pointsway(b,0)-pointsway(b+1,0)))))
            xd=x0-np.sign(pointsway(b,1)-pointsway(b+1,1))*(abs((pointsway(b,1)-pointsway(b+1,1))/(pointsway(b,0)-pointsway(b+1,0))))*\
            (y0-yd)

        elif b==len(self.pointsway)-1:

            A=self.pointsway(b,1)-self.pointsway(b-1,1)
            B=self.pointsway(b-1,0)-self.pointsway(b,0)
            C=self.pointsway(b,0)*self.pointsway(b-1,1)-self.pointsway(b-1,0)*self.pointsway(b,1)
            #求垂足
            y0=(B*B*realposition[0,1]-A*B*realposition[0,0]-A*C)/(A*A+B*B)
            x0=(-A*B*realposition[0,1]+A*A*realposition[0,0]-B*C)/(A*A+B*B)
            yd=y0-np.sign(pointsway(b-1,0)-pointsway(b,0))*np.sqrt(np.square(self.deta)/np.square(1+(abs(pointsway(b-1,1)-pointsway(b,1))/\
            (pointsway(b-1,0)-pointsway(b,0)))))
            xd=x0-np.sign(pointsway(b-1,1)-pointsway(b,1))*(abs((pointsway(b-1,1)-pointsway(b,1))/(pointsway(b-1,0)-pointsway(b,0))))*\
            (y0-yd)     

        else:

            PbPt = realposition-self.pointsway(b)
            PbPb_1 = self.pointsway(b-1)-self.pointsway(b)
            PbPb_1 = self.pointsway(b+1)-self.pointsway(b)

            if self.angle(PbPt, PbPb_1) < self.angle(PbPt, PbPb1):

                A=self.pointsway(b,1)-self.pointsway(b-1,1)
                B=self.pointsway(b-1,0)-self.pointsway(b,0)
                C=self.pointsway(b,0)*self.pointsway(b-1,1)-self.pointsway(b-1,0)*self.pointsway(b,1)
                #求垂足
                y0=(B*B*realposition[0,1]-A*B*realposition[0,0]-A*C)/(A*A+B*B)
                x0=(-A*B*realposition[0,1]+A*A*realposition[0,0]-B*C)/(A*A+B*B)
                yd=y0-np.sign(pointsway(b-1,0)-pointsway(b,0))*np.sqrt(np.square(self.deta)/np.square(1+(abs(pointsway(b-1,1)-pointsway(b,1))/\
                (pointsway(b-1,0)-pointsway(b,0)))))
                xd=x0-np.sign(pointsway(b-1,1)-pointsway(b,1))*(abs((pointsway(b-1,1)-pointsway(b,1))/(pointsway(b-1,0)-pointsway(b,0))))*\
                (y0-yd) 

            else:

                A=self.pointsway(b+1,1)-self.pointsway(b,1)
                B=self.pointsway(b,0)-self.pointsway(b+1,0)
                C=self.pointsway(b+1,0)*self.pointsway(b,1)-self.pointsway(b,0)*self.pointsway(b+1,1)
                #求垂足
                y0=(B*B*realposition[0,1]-A*B*realposition[0,0]-A*C)/(A*A+B*B)
                x0=(-A*B*realposition[0,1]+A*A*realposition[0,0]-B*C)/(A*A+B*B)
                yd=y0-np.sign(pointsway(b,0)-pointsway(b+1,0))*np.sqrt(np.square(self.deta)/np.square(1+(abs(pointsway(b,1)-pointsway(b+1,1))/\
                (pointsway(b,0)-pointsway(b+1,0)))))
                xd=x0-np.sign(pointsway(b,1)-pointsway(b+1,1))*(abs((pointsway(b,1)-pointsway(b+1,1))/(pointsway(b,0)-pointsway(b+1,0))))*\
                (y0-yd)                

        self.set_point = np.array([xd,yd]) 
        return (self.set_point)

    def angle(self,a,b):
        c = np.dot(a,b)
        d = np.dot(a,a)
        e = np.sqrt(d)
        f = np.dot(b,b)
        g = np.aqrt(f)
        h = c/(e*g)
        z = acos(h)
    return z

    def callback(self,msg):
        '''path_following Callback Function'''
        rospy.loginfo("wave glider position:: %s", msg.data)
        
        while not rospy.is_shutdown():

            self.set_point=self.p_f(msg.data)

            self.course_desired=self.p_s(self.set_point[0,0],self.set_point[0,1],msg.data[0,0],msg.data[0,1])

            self.pub.publish(self.course_desired)
            
 

if __name__ == '__main__':
    try:
        Path_following()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass