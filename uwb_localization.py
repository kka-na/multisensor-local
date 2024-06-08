import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter, lfilter_zi

from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from uwb_driver.msg import UwbRange
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Point
from visualization_msgs.msg import Marker
import tf.transformations as tf_trans
import math
import tf2_ros

from kalman import KalmanFilter

class UWBLocalization:
    def __init__(self):
        rospy.init_node('uwb_localization', anonymous=True)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.uwb200a = {'1': None, '2': None, '3': None}
        self.uwb200b = {'1': None, '2': None, '3': None}
        self.uwb201a = {'1': None, '2': None, '3': None}
        self.uwb201b = {'1': None, '2': None, '3': None}
        self.attribute_mapping = {
            (200, 0): 'uwb200a',
            (200, 1): 'uwb200b',
            (201, 0): 'uwb201a',
            (201, 1): 'uwb201b'
        }
        self.responder_mapping = {
            100: '1',
            101: '2',
            102: '3'
        }
        self.uwb_cnt = 0
        self.avg_position = [0,0,0]
        self.kf_x = KalmanFilter(state_dim=2, meas_dim=1)
        self.kf_y = KalmanFilter(state_dim=2, meas_dim=1)
        self.kf_z = KalmanFilter(state_dim=2, meas_dim=1)
        for kf in [self.kf_x, self.kf_y, self.kf_z]:
            kf.F = np.array([[1, 0.05], [0, 1]])
            kf.H = np.array([[1, 0]])
            kf.Q = np.array([[1e-4, 0], [0, 1e-3]])
            kf.R = np.array([[1e-1]])
            kf.x = np.array([[0], [0]])
         # 저주파 필터용 IMU 데이터 버퍼
        self.imu_buffer = {'x': [], 'y': [], 'z': []}
        self.buffer_size = 5  # 이동 평균 필터의 크기


        rospy.Subscriber("/uwb_endorange_info", UwbRange, self.callback_uwb)
        self.uwb_marker_pub = rospy.Publisher("/uwb_marker", Marker, queue_size=1)
        rospy.Subscriber('/imu/imu', Imu, self.imu_cb)
        rospy.Subscriber('/os1_cloud_node1/points', PointCloud2, self.points1_cb)
        rospy.Subscriber('/os1_cloud_node2/points', PointCloud2, self.points2_cb)
        rospy.Subscriber('/node_pos_marker_sc', Marker, self.node_marker_cb)
        self.transform_points1_pub = rospy.Publisher('/os1_cloud_node1/transformed_points', PointCloud2, queue_size=1)
        self.transform_points2_pub = rospy.Publisher('/os1_cloud_node2/transformed_points', PointCloud2, queue_size=1)
        self.large_node_marker_pub = rospy.Publisher('/node_pos/large', Marker, queue_size=10)
        self.orientation = Quaternion()
        self.linear_accel = np.zeros(3)
        self.stamp = rospy.Time.now()
    
    def imu_cb(self, msg):
        self.orientation = msg.orientation        
        ax = 0.1*msg.linear_acceleration.y
        ay = 0.1*msg.linear_acceleration.x
        az = 0.1*msg.linear_acceleration.z

         # 이동 평균 필터 적용
        self.imu_buffer['x'].append(ax)
        self.imu_buffer['y'].append(ay)
        self.imu_buffer['z'].append(az)

        if len(self.imu_buffer['x']) > self.buffer_size:
            self.imu_buffer['x'].pop(0)
            self.imu_buffer['y'].pop(0)
            self.imu_buffer['z'].pop(0)

        ax_filtered = np.mean(self.imu_buffer['x'])
        ay_filtered = np.mean(self.imu_buffer['y'])
        az_filtered = np.mean(self.imu_buffer['z'])

        # self.kf_x.predict()
        # self.kf_y.predict()
        # self.kf_z.predict()

        # self.kf_x.update(np.array([[ax_filtered]]))
        # self.kf_y.update(np.array([[ay_filtered]]))
        # self.kf_z.update(np.array([[az_filtered]]))

        # x_state = self.kf_x.get_state()
        # y_state = self.kf_y.get_state()
        # z_state = self.kf_z.get_state()

        # corrected_x = self.avg_position[0] + x_state[0, 0]
        # corrected_y = self.avg_position[1] + y_state[0, 0]
        # corrected_z = self.avg_position[2] + z_state[0, 0]

        corrected_x = self.avg_position[0] + ax_filtered
        corrected_y = self.avg_position[1] + ay_filtered
        corrected_z = self.avg_position[2] + az_filtered

        pos = Point(corrected_x, corrected_y, -corrected_z)
        # pos = Point(self.avg_position[0], self.avg_position[1], self.avg_position[2])

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'gt'
        transform.transform.translation = pos
        original_quat_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(original_quat_list)
        yaw = -yaw + 90
        new_quat_list = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        normalized_quaternion = Quaternion(*new_quat_list)
        try:
            normalized_quaternion = self.normalize_quaternion(normalized_quaternion)
            transform.transform.rotation = normalized_quaternion
        except ValueError as e:
            rospy.logerr(f"Invalid quaternion: {e}")
            return
        self.tf_broadcaster.sendTransform(transform)

        marker = self.getMarker_ego('uwb_marker', self.uwb_cnt)
        self.uwb_marker_pub.publish(marker)
        self.uwb_cnt += 1

    def callback_uwb(self, data: UwbRange):
        uwb_data = [data.distance, data.responder_location.x, data.responder_location.y, (data.responder_location.z-1.5), data.rspd_antenna_offset.x, data.rspd_antenna_offset.y, data.rspd_antenna_offset.z]
        key = (data.requester_id, data.antenna)
        self.stamp = data.header.stamp
        if key in self.attribute_mapping and data.responder_id in self.responder_mapping:
            attribute_name = self.attribute_mapping[key]
            responder_key = self.responder_mapping[data.responder_id]
            if hasattr(self, attribute_name):
                getattr(self, attribute_name)[responder_key] = uwb_data

        # UWB 데이터를 사용하여 위치 보정
        if not self.has_none_value():
            self.calc_local_position()

    def has_none_value(self):
        uwb_dicts = [self.uwb200a, self.uwb200b, self.uwb201a, self.uwb201b]
        for uwb_dict in uwb_dicts:
            for key, value in uwb_dict.items():
                if value is None:
                    return True
        return False

    def calc_local_position(self):
        if self.has_none_value():
            return
        pos1 = self.trilaterate(self.uwb200a)
        pos2 = self.trilaterate(self.uwb200b)
        pos3 = self.trilaterate(self.uwb201a)
        pos4 = self.trilaterate(self.uwb201b)
        positions = [pos1, pos2, pos3, pos4]
        self.avg_position = self.calculate_average_position(positions)

    def trilaterate(self, uwb):
        uwb1 = uwb['1']
        uwb2 = uwb['2']
        uwb3 = uwb['3']
        r1, p1 = uwb1[0], uwb1[1:4]
        r2, p2 = uwb2[0], uwb2[1:4]
        r3, p3 = uwb3[0], uwb3[1:4]

        P1 = np.array(p1)
        P2 = np.array(p2)
        P3 = np.array(p3)
        
        ex = (P2 - P1) / np.linalg.norm(P2 - P1)
        i = np.dot(ex, P3 - P1)
        ey = (P3 - P1 - i * ex) / np.linalg.norm(P3 - P1 - i * ex)
        ez = np.cross(ex, ey)
        d = np.linalg.norm(P2 - P1)
        j = np.dot(ey, P3 - P1)
        
        x = (r1**2 - r2**2 + d**2) / (2 * d)
        y = (r1**2 - r3**2 + i**2 + j**2) / (2 * j) - (i/j) * x
        z_square = r1**2 - x**2 - y**2
        if z_square < 0:
            return [0, 0, 0]
        z = np.sqrt(z_square)
        
        estimated_position = P1 + x * ex + y * ey + z * ez
        return estimated_position
    
    def calculate_average_position(self, positions):
        positions_array = np.array(positions)
        if len(positions_array) == 0 or np.all(positions_array == 0):
            return self.avg_position

        # 0이 아닌 값들만 추출하여 평균 계산
        non_zero_positions = positions_array[positions_array != 0].reshape(-1, 3)
        if len(non_zero_positions) == 0:
            return self.avg_postiion
        average_position = np.mean(non_zero_positions, axis=0)
        return average_position

    def normalize_quaternion(self, quat):
        norm = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
        if norm == 0:
            raise ValueError("Quaternion has zero length, can't be normalized.")
        quat.x /= norm
        quat.y /= norm
        quat.z /= norm
        quat.w /= norm
        return quat

    def getMarker_ego(self, _ns, _id):
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
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        return marker

    def node_marker_cb(self, msg):
        marker = msg
        marker.scale.x = 3
        marker.scale.y = 3
        marker.scale.z = 1
        yellow_color = Marker().color
        yellow_color.r = 1.0
        yellow_color.g = 1.0
        yellow_color.b = 0.0
        yellow_color.a = 1.0
        marker.colors = [yellow_color] * 3
        self.large_node_marker_pub.publish(marker)
    
    def points1_cb(self, msg):
        modified_points = []
        for point in pc2.read_points(msg, field_names=None, skip_nans=True):
            point_list = list(point)
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
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0]
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

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.calc_local_position()
            rate.sleep()

if __name__ == '__main__':
    try:
        uwb_localization = UWBLocalization()
        uwb_localization.execute()
    except rospy.ROSInterruptException:
        pass