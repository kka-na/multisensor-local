import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import  PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from uwb_driver.msg import UwbRange
from geometry_msgs.msg import PoseStamped, Quaternion,TransformStamped, Point
from visualization_msgs.msg import Marker
import tf.transformations as tf_trans
import math
import tf2_ros


class LeicaGT:
    def __init__(self):
        rospy.init_node('leica_gt', anonymous=True)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/imu/imu", Imu, self.imu_cb)
        rospy.Subscriber("/leica/pose/relative", PoseStamped, self.leica_cb)
        rospy.Subscriber('/os1_cloud_node1/points', PointCloud2, self.points1_cb)
        rospy.Subscriber('/os1_cloud_node2/points', PointCloud2, self.points2_cb)
        self.leica_marker_pub = rospy.Publisher('/leica/marker', Marker, queue_size=1)
        self.transform_points1_pub = rospy.Publisher('/os1_cloud_node1/transformed_points', PointCloud2, queue_size=1)
        self.transform_points2_pub = rospy.Publisher('/os1_cloud_node2/transformed_points', PointCloud2, queue_size=1)
        self.leica_cnt = 0
        self.orientation = Quaternion()
        rospy.spin()

    def imu_cb(self, msg):
        self.orientation = msg.orientation

    def leica_cb(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'world'  # 부모 프레임 ID
        transform.child_frame_id = 'gt'  # 자식 프레임 ID
        transform.transform.translation = msg.pose.position
        original_quat_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(original_quat_list)
        yaw = -yaw+90
        new_quat_list = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        normalized_quaternion = Quaternion(*new_quat_list)
        try:
            normalized_quaternion = self.normalize_quaternion(normalized_quaternion)
            transform.transform.rotation = normalized_quaternion
        except ValueError as e:
            rospy.logerr(f"Invalid quaternion: {e}")
            return
        self.tf_broadcaster.sendTransform(transform)
        marker = self.getMarker_ego('leica_gt', self.leica_cnt, msg.pose.position)
        self.leica_marker_pub.publish(marker)
        self.leica_cnt += 1

    def normalize_quaternion(self, quat):
        norm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if norm == 0:
            raise ValueError("Quaternion has zero length, can't be normalized.")
        quat.x /= norm
        quat.y /= norm
        quat.z /= norm
        quat.w /= norm
        return quat

    def getMarker_ego(self, _ns, _id, position):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'gt'
        marker.ns = _ns
        marker.id = _id
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.pose.position = Point(0,0,0)
        roll, pitch, yaw = tf_trans.euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        yaw += 90
        new_quat_list = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        normalized_quaternion = Quaternion(*new_quat_list)
        normalized_quaternion = self.normalize_quaternion(normalized_quaternion)
        marker.pose.orientation = normalized_quaternion #$Quaternion(0,0,0,1)
        return marker
    
    def points1_cb(self, msg):
        modified_points = []
        for point in pc2.read_points(msg, field_names=None, skip_nans=True):
            point_list = list(point)  # 튜플을 리스트로 변환
            point_list[2] = -point_list[2]
            point_list[0] = -point_list[0]
            modified_points.append(point_list)
        header = msg.header
        header.frame_id = 'gt'
        new_msg = pc2.create_cloud(header, msg.fields, modified_points)
        self.transform_points1_pub.publish(new_msg)

    def points2_cb(self, msg):
        modified_points = []
        rotation_matrix = np.array([
            [1.0,  0.0,  0.0],
            [ 0.0,  0.0,  1.0],
            [ 0.0,  -1.0,  0.0]
        ])
        for point in pc2.read_points(msg, field_names=None, skip_nans=True):
            original_point = np.array([point[0], point[1], point[2]])
            rotated_point = np.dot(rotation_matrix, original_point)
            modified_point = rotated_point.tolist() + list(point[3:])
            modified_points.append(modified_point)
        header = msg.header
        header.frame_id = 'gt'
        new_msg = pc2.create_cloud(header, msg.fields, modified_points)
        self.transform_points2_pub.publish(new_msg)

if __name__ == '__main__':

    try:
        leica_gt = LeicaGT()
    except rospy.ROSInterruptException:
        pass
