#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
import time 

csv_file_1 = open('./csv/sbs01_uwb.csv', 'w', newline='')
csv_file_2 = open('./csv/sbs01_uwb_lio.csv', 'w', newline='')

csv_writer_1 = csv.writer(csv_file_1)
csv_writer_2 = csv.writer(csv_file_2)

header = [
    'time', 'field.header.seq', 'field.header.stamp', 
    'field.pose.pose.position.x', 'field.pose.pose.position.y', 'field.pose.pose.position.z',
    'field.pose.pose.orientation.x', 'field.pose.pose.orientation.y', 'field.pose.pose.orientation.z', 'field.pose.pose.orientation.w'
]
csv_writer_1.writerow(header)
csv_writer_2.writerow(header)

def callback_odom1(msg):
    row = [
        time.time_ns(), 
        msg.header.seq,
        time.time_ns(),  # ROS time을 seconds로 변환합니다.
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    csv_writer_1.writerow(row)

def callback_odom2(msg):
    row = [
        time.time_ns(), 
        msg.header.seq,
        time.time_ns(),  # ROS time을 seconds로 변환합니다.
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    csv_writer_2.writerow(row)

def listener():
    rospy.init_node('odometry_listener', anonymous=True)

    rospy.Subscriber('/uwb_odom', Odometry, callback_odom1)
    rospy.Subscriber('/correct_uwb_odom', Odometry, callback_odom2)

    rospy.spin()

    csv_file_1.close()
    csv_file_2.close()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
