#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
import time

# CSV 파일을 열고 기록할 준비를 합니다.
#csv_file_1 = open('./csv/nya01_fast_livo.csv', 'w', newline='')  # 첫 번째 토픽의 데이터 파일
csv_file_2 = open('./csv/eee01_uwb.csv', 'w', newline='')  # 두 번째 토픽의 데이터 파일

# CSV 작성자를 생성합니다.
#csv_writer_1 = csv.writer(csv_file_1)
csv_writer_2 = csv.writer(csv_file_2)

# CSV 파일에 헤더를 작성합니다.
header = [
    'time', 'field.header.seq', 'field.header.stamp', 
    'field.pose.pose.position.x', 'field.pose.pose.position.y', 'field.pose.pose.position.z',
    'field.pose.pose.orientation.x', 'field.pose.pose.orientation.y', 'field.pose.pose.orientation.z', 'field.pose.pose.orientation.w'
]
#csv_writer_1.writerow(header)
csv_writer_2.writerow(header)

def callback_odom1(msg):
    row = [
        msg.header.stamp.to_sec(), 
        msg.header.seq,
        msg.header.stamp.to_sec(),  # ROS time을 seconds로 변환합니다.
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
    # 두 번째 토픽에서 수신한 데이터를 CSV 파일에 기록합니다.
    row = [
        msg.header.stamp.to_sec(), 
        msg.header.seq,
        msg.header.stamp.to_sec(),  # ROS time을 seconds로 변환합니다.
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

    #rospy.Subscriber('/Odometry', Odometry, callback_odom1)
    rospy.Subscriber('/uwb_odom', Odometry, callback_odom2)

    rospy.spin()

    #csv_file_1.close()
    csv_file_2.close()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
