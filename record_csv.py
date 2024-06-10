#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
import time

# CSV 파일을 열고 기록할 준비를 합니다.
csv_file_1 = open('./csv/nya01_fast_livo.csv', 'w', newline='')  # 첫 번째 토픽의 데이터 파일
csv_file_2 = open('./csv/nya01_uwb.csv', 'w', newline='')  # 두 번째 토픽의 데이터 파일

# CSV 작성자를 생성합니다.
csv_writer_1 = csv.writer(csv_file_1)
csv_writer_2 = csv.writer(csv_file_2)

# CSV 파일에 헤더를 작성합니다.
header = [
    'time', 'field.header.seq', 'field.header.stamp', 
    'field.pose.pose.position.x', 'field.pose.pose.position.y', 'field.pose.pose.position.z',
    'field.pose.pose.orientation.x', 'field.pose.pose.orientation.y', 'field.pose.pose.orientation.z', 'field.pose.pose.orientation.w',
    'field.twist.twist.linear.x', 'field.twist.twist.linear.y', 'field.twist.twist.linear.z',
    'field.twist.twist.angular.x', 'field.twist.twist.angular.y', 'field.twist.twist.angular.z'
]
csv_writer_1.writerow(header)
csv_writer_2.writerow(header)

def callback_odom1(msg):
    # 첫 번째 토픽에서 수신한 데이터를 CSV 파일에 기록합니다.
    current_time = time.time()  # 현재 시간을 기록합니다.
    row = [
        current_time,
        msg.header.seq,
        msg.header.stamp.to_sec(),  # ROS time을 seconds로 변환합니다.
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z
    ]
    csv_writer_1.writerow(row)

def callback_odom2(msg):
    # 두 번째 토픽에서 수신한 데이터를 CSV 파일에 기록합니다.
    current_time = time.time()  # 현재 시간을 기록합니다.
    row = [
        current_time,
        msg.header.seq,
        msg.header.stamp.to_sec(),  # ROS time을 seconds로 변환합니다.
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z
    ]
    csv_writer_2.writerow(row)

def listener():
    rospy.init_node('odometry_listener', anonymous=True)

    # 두 개의 /Odometry 토픽을 구독합니다.
    rospy.Subscriber('/Odometry', Odometry, callback_odom1)
    #rospy.Subscriber('/uwb_odom', Odometry, callback_odom2)

    rospy.spin()

    # 노드가 종료될 때 CSV 파일을 닫습니다.
    csv_file_1.close()
    csv_file_2.close()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
