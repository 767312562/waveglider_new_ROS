#!/usr/bin/env python
#encoding:utf-8  
'''test ROS Node'''
# license removed for brevity
import rospy
import numpy as np
import sys
import json
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, QCoreApplication, QDateTime
from RMS import Ui_RMS
from gui.msg import pf
from std_msgs.msg import Float32MultiArray
from CoordinateTransfer import millerToXY
from std_msgs.msg import String, Int8



class RMS_show(QMainWindow,Ui_RMS):

    def __init__(self,parent=None):
        super(RMS_show,self).__init__(parent)
        self.setupUi(self)
        self.PrepWidgets()
        self.Timer=QTimer()
        self.CallBackFunctions()
        self.textEdit.setText("Please input setpoint or waypoints")
        #创建节点
        rospy.init_node('RMS_UI', anonymous=True)
        #创建一个flag
        self.flag_pub = rospy.Publisher('/flag', Int8, queue_size=10)
        #Realposition订阅者创建
        rospy.Subscriber("/position_real", Float32MultiArray, self.Position_show)
        #positionkeeping发布者创建
        self.positionkeeping_pub = rospy.Publisher('/set_point', Float32MultiArray, queue_size=10)
        #pathfollowing发布者创建
        self.pathfollowing_pub = rospy.Publisher('/waypoints', pf, queue_size=10)
        #创建一个启动螺旋桨的flag
        self.switch_pub = rospy.Publisher('/switch', Int8, queue_size=10)

    def PrepWidgets(self):
        self.checkBox_1.setEnabled(False)
        self.checkBox_2.setEnabled(False)
        self.PushButton_start.setEnabled(True)
        self.PushButton_suspend.setEnabled(True)
        self.PushButton_save.setEnabled(False)
        self.PushButton_quit.setEnabled(False)
        self.PositionKeeping_Lat.setEnabled(True)
        self.PositionKeeping_Lon.setEnabled(True)
        self.Point1_Lat.setEnabled(True)
        self.Point1_Lon.setEnabled(True)
        self.Point2_Lat.setEnabled(True)
        self.Point2_Lon.setEnabled(True)
        self.Point3_Lat.setEnabled(True)
        self.Point3_Lon.setEnabled(True)
        self.Point4_Lat.setEnabled(True)
        self.Point4_Lon.setEnabled(True)
        self.Point5_Lat.setEnabled(True)
        self.Point5_Lon.setEnabled(True)
        self.Lat_real.setEnabled(False)
        self.Lon_real.setEnabled(False)
        self.Date.setEnabled(False)
        self.PushButton_quit.setText('Quit')

    def Start(self):
        self.PushButton_start.setEnabled(False)
        self.PushButton_suspend.setEnabled(True)
        #self.PushButton_save.setEnabled(True)
        self.PushButton_quit.setEnabled(True)
        self.checkBox_1.setEnabled(True)
        self.checkBox_2.setEnabled(True)
        self.PositionKeeping_Lat.setEnabled(False)
        self.PositionKeeping_Lon.setEnabled(False)
        self.Point1_Lat.setEnabled(False)
        self.Point1_Lon.setEnabled(False)
        self.Point2_Lat.setEnabled(False)
        self.Point2_Lon.setEnabled(False)
        self.Point3_Lat.setEnabled(False)
        self.Point3_Lon.setEnabled(False)
        self.Point4_Lat.setEnabled(False)
        self.Point4_Lon.setEnabled(False)
        self.Point5_Lat.setEnabled(False)
        self.Point5_Lon.setEnabled(False)
        self.PushButton_quit.setText('Quit')
        self.Timer.start(1000)
  
    def Showtime(self):
        time = QDateTime.currentDateTime()
        timeDisplay = time.toString("yyyy-MM-dd hh:mm:ss")
        self.Date.setText(timeDisplay)
    
    def Publishfun(self):
        #用于启动和停止螺旋桨
        if self.PushButton_quit.text() == 'Quit':
            switch_flag = 1
            self.switch_pub.publish(switch_flag)
            rospy.loginfo(switch_flag)

        elif self.PushButton_quit.text() == 'Active':
            switch_flag = 0
            self.switch_pub.publish(switch_flag)
            rospy.loginfo(switch_flag)
          

        if self.checkBox_1.isChecked():
            self.flag = 1
            self.flag_pub.publish(self.flag)
            setpoint = millerToXY(float(self.PositionKeeping_Lon.text()),float(self.PositionKeeping_Lat.text()))
            setpoint_1 = [-setpoint[1],setpoint[0]]
            setpoint_xy = Float32MultiArray(data=setpoint_1)
            rospy.loginfo(setpoint_xy.data)
            self.positionkeeping_pub.publish(setpoint_xy)
            self.textEdit.setText("The Positionkeeping point is set successfully")

        elif self.checkBox_2.isChecked():
            self.flag = 2
            self.flag_pub.publish(self.flag)
            Point1_xy = millerToXY(float(self.Point1_Lon.text()),float(self.Point1_Lat.text()))               
            Point2_xy = millerToXY(float(self.Point2_Lon.text()),float(self.Point2_Lat.text()))
            Point3_xy = millerToXY(float(self.Point3_Lon.text()),float(self.Point3_Lat.text()))
            Point4_xy = millerToXY(float(self.Point4_Lon.text()),float(self.Point4_Lat.text()))
            Point5_xy = millerToXY(float(self.Point5_Lon.text()),float(self.Point5_Lat.text()))
            pfpoint1 = [Point1_xy[0],-Point1_xy[1]]
            pfpoint2 = [Point2_xy[0],-Point2_xy[1]]
            pfpoint3 = [Point3_xy[0],-Point3_xy[1]]
            pfpoint4 = [Point4_xy[0],-Point4_xy[1]]
            pfpoint5 = [Point5_xy[0],-Point5_xy[1]]
            waypoints_xy = pf()
            
            waypoints_xy.p_1 = pfpoint1
            waypoints_xy.p_2 = pfpoint2
            waypoints_xy.p_3 = pfpoint3
            waypoints_xy.p_4 = pfpoint4
            waypoints_xy.p_5 = pfpoint5
            
            self.pathfollowing_pub.publish(waypoints_xy)
            rospy.loginfo("path following! the way_points are: %s %s %s %s %s", waypoints_xy.p_1, waypoints_xy.p_2, waypoints_xy.p_3, waypoints_xy.p_4, waypoints_xy.p_5)
            self.textEdit.setText("The pointsway is set successfully")
        else:
            pass


    def Position_show(self,data):
        Lon_real_get = data.data[0]
        Lat_real_get = data.data[1]
        position=[Lon_real_get,Lat_real_get]
        position_real='position_real.json'
        with open(position_real,'a') as position_obj:
            position_obj.write('\n'+str(position))
        self.Lat_real.setText(str(Lat_real_get))
        self.Lon_real.setText(str(Lon_real_get))

    def Suspend(self):
        self.Timer.stop()
        self.PushButton_start.setEnabled(True)
        self.PushButton_quit.setEnabled(True)
        self.PushButton_suspend.setEnabled(False)
        self.PositionKeeping_Lat.setEnabled(True)
        self.PositionKeeping_Lon.setEnabled(True)
        self.Point1_Lat.setEnabled(True)
        self.Point1_Lon.setEnabled(True)
        self.Point2_Lat.setEnabled(True)
        self.Point2_Lon.setEnabled(True)
        self.Point3_Lat.setEnabled(True)
        self.Point3_Lon.setEnabled(True)
        self.Point4_Lat.setEnabled(True)
        self.Point4_Lon.setEnabled(True)
        self.Point5_Lat.setEnabled(True)
        self.Point5_Lon.setEnabled(True)

    def Quit(self):
        tag = self.PushButton_quit.text()
        if tag == 'Active':
            self.PushButton_quit.setText('Quit')
        if tag == 'Quit':
            self.PushButton_quit.setText('Active')

    def CallBackFunctions(self):
        self.Timer.timeout.connect(self.Showtime)
        self.Timer.timeout.connect(self.Publishfun)
        self.PushButton_start.clicked.connect(self.Start)
        self.PushButton_quit.clicked.connect(self.Quit)
        self.PushButton_suspend.clicked.connect(self.Suspend)


if __name__ == '__main__':
        try:
            app = QApplication(sys.argv)
            ui=RMS_show()
            ui.show()
            sys.exit(app.exec_())
        except rospy.ROSInterruptException:
            pass
