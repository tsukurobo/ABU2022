#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from numpy import math
import rospy
#from abu2022_msgs.msg import BaseData, BaseCmd

from abu2022_msgs.msg import BaseCmd
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class VirtualOmni:
    def __init__(self):
        #self.enc_data = BaseData()
        self.curr_vx = 0
        self.curr_vy = 0
        self.curr_omega = 0
        self.curr_yaw = 0
        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = "base_link"
        self.ts.transform.translation.x = 0.5
        self.ts.transform.translation.y = 6.0

        self.tb = TransformBroadcaster()
        # self.base_data_pub = \
        #         rospy.Publisher('omni_info', BaseData, queue_size = 10)
        self.cmd_sub = \
                rospy.Subscriber('cmd', BaseCmd, self.on_receive_cmd)
        #frequency = float(rospy.get_param('/const/spec/frequency'))
        self.frequency = 100
        self.rate = rospy.Rate(self.frequency)

    def on_receive_cmd(self, cmd_msg):
        # self.enc_data.delta_x = cmd_msg.vx*1.0/self.frequency
        # self.enc_data.delta_y = cmd_msg.vy*1.0/self.frequency
        # self.enc_data.delta_theta = cmd_msg.omega*1.0/self.frequency
        self.curr_vx = cmd_msg.vx
        self.curr_vy = cmd_msg.vy
        self.curr_omega = cmd_msg.omega

    def start(self):
        while not rospy.is_shutdown():
            self.ts.header.stamp = rospy.Time.now()
            delta_x = self.curr_vx*1.0/self.frequency
            delta_y = self.curr_vy*1.0/self.frequency
            self.ts.transform.translation.x += delta_x*math.cos(self.curr_yaw) - delta_y*math.sin(self.curr_yaw)
            self.ts.transform.translation.y += delta_x*math.sin(self.curr_yaw) + delta_y*math.cos(self.curr_yaw)
            self.curr_yaw += self.curr_omega*1.0/self.frequency
            self.ts.transform.rotation.z = math.sin(self.curr_yaw)
            self.ts.transform.rotation.w = math.cos(self.curr_yaw)
            self.tb.sendTransform(self.ts)
            #self.raw_encoder_pub.publish(self.enc_data)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('virtual_omni')
    vo = VirtualOmni()
    vo.start()

