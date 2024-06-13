import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import ColorRGBA
from uwb_driver.msg import UwbRange
from geometry_msgs.msg import Quaternion, TransformStamped, Point, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import tf2_ros

from ukf import UnscentedKalmanFilter

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
        self.attribute_mapping = {(200, 0): 'uwb200a',(200, 1): 'uwb200b',(201, 0): 'uwb201a',(201, 1): 'uwb201b'}
        self.responder_mapping = {100: '1',101: '2',102: '3'}

        self.uwb_cnt = 0
        self.leica_cnt = 0
        self.avg_position = None
        self.orientation = Quaternion()
        self.stamp = rospy.Time.now()
        self.initial_pos = None
        self.initializing = True
        self.init_duration = rospy.Duration(5)  # 안정화 시간 5초
        self.init_start_time = rospy.Time.now()

        process_noise = np.eye(6) * 0.01  # 과정 노이즈 공분산 행렬
        measurement_noise = np.eye(3) * 0.5  # 측정 노이즈 공분산 행렬
        self.ukf = UnscentedKalmanFilter(0.01, process_noise, measurement_noise)

        rospy.Subscriber("/uwb_endorange_info", UwbRange, self.callback_uwb)
        rospy.Subscriber("/leica/pose/relative", PoseStamped, self.leica_cb)
        self.uwb_marker_pub = rospy.Publisher("/uwb_marker", Marker, queue_size=1)
        rospy.Subscriber('/imu/imu', Imu, self.imu_cb)
        rospy.Subscriber('/os1_cloud_node1/points', PointCloud2, self.points1_cb)
        rospy.Subscriber('/os1_cloud_node2/points', PointCloud2, self.points2_cb)
        rospy.Subscriber('/node_pos_marker_sc', Marker, self.node_marker_cb)
        self.leica_marker_pub = rospy.Publisher('/leica/marker', Marker, queue_size=1)
        self.transform_points1_pub = rospy.Publisher('/os1_cloud_node1/transformed_points', PointCloud2, queue_size=1)
        self.transform_points2_pub = rospy.Publisher('/os1_cloud_node2/transformed_points', PointCloud2, queue_size=1)
        self.large_node_marker_pub = rospy.Publisher('/node_pos/large', Marker, queue_size=10)
        self.uwb_odom_pub = rospy.Publisher('/uwb_odom',Odometry, queue_size=1)

    def imu_cb(self, msg):
        self.orientation = Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        if self.avg_position is None:
            return
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        current_time = msg.header.stamp
        dt = (current_time - self.stamp).to_sec()
        if dt >= 0.01:
            self.stamp = current_time

            # self.kalman_filter.integrate_imu(accel, dt)
            # self.kalman_filter.predict(dt)

            self.ukf.predict(accel)
            self.ukf.update(np.array(self.avg_position), accel)
            ukf_position = self.ukf.get_state()
            
            if self.initial_pos is None:
                self.initial_pos = ukf_position[:3]
            self.transform_to_local_frame(ukf_position[:3])

            #self.publish_uwb(Point(*ukf_position[:3]), self.orientation)

        # self.avg_position = self.kalman_filter.state[:3]
        # self.publish_uwb(Point(*self.avg_position), self.orientation)

    def calc_world_position(self):
        if self.has_none_value():
            return
        pos1 = self.trilaterate(self.uwb200a)
        pos2 = self.trilaterate(self.uwb200b)
        pos3 = self.trilaterate(self.uwb201a)
        pos4 = self.trilaterate(self.uwb201b)
        positions = [pos1, pos2, pos3, pos4]
        avg_position = self.calculate_average_position(positions)
        if avg_position is not None:
            self.avg_position = [-avg_position[1], -avg_position[0], abs(avg_position[2])]

        # avg_position -> world coordinate
        # self.kalman_filter.update(self.avg_position)
        # self.avg_position = self.kalman_filter.state[:3]
        # self.publish_uwb(Point(*self.avg_position), self.orientation)
        
        #self.transform_to_local_frame()
    

    def callback_uwb(self, data: UwbRange):
        uwb_data = [data.distance, data.responder_location.x, data.responder_location.y, data.responder_location.z, data.rspd_antenna_offset.x, data.rspd_antenna_offset.y, data.rspd_antenna_offset.z]
        key = (data.requester_id, data.antenna)
        self.stamp = data.header.stamp
        if key in self.attribute_mapping and data.responder_id in self.responder_mapping:
            attribute_name = self.attribute_mapping[key]
            responder_key = self.responder_mapping[data.responder_id]
            if hasattr(self, attribute_name):
                getattr(self, attribute_name)[responder_key] = uwb_data

    def leica_cb(self, msg):
        marker = self.getMarker_target('leica_gt', self.leica_cnt, msg.pose.position, self.orientation)
        self.leica_marker_pub.publish(marker)
        self.leica_cnt += 1


    def transform_to_local_frame(self, current_pos):
        translation = current_pos - self.initial_pos
        quat_list = [self.orientation.x, self.orientation.y,self.orientation.z, self.orientation.w]
        rotation = R.from_quat(quat_list)
        rotation_matrix = rotation.as_matrix()

        centered_translation = translation - translation
        rotated_centered_translation = np.dot(rotation_matrix.T, centered_translation)
        rotated_pos = rotated_centered_translation + translation
        inv_rotation_matrix = rotation_matrix.T
        local_quat = R.from_matrix(inv_rotation_matrix).as_quat()

        _pos = Point(*rotated_pos)
        _ori = Quaternion(*local_quat)

        self.publish_uwb(_pos, _ori)

    def publish_uwb(self, position, orientation):
        odom = Odometry()
        odom.pose.pose.position = position
        odom.pose.pose.orientation = orientation

        transform = TransformStamped()
        transform.header.stamp = self.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'local'
        transform.transform.translation = position
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)

        marker = self.getMarker_ego('uwb_marker', self.uwb_cnt, position, orientation)
        self.uwb_marker_pub.publish(marker)
        self.uwb_odom_pub.publish(odom)
        self.uwb_cnt += 1


    def has_none_value(self):
        uwb_dicts = [self.uwb200a, self.uwb200b, self.uwb201a, self.uwb201b]
        for uwb_dict in uwb_dicts:
            for key, value in uwb_dict.items():
                if value is None:
                    return True
        return False

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
            return None

        # # 0이 아닌 값들만 추출하여 평균 계산
        non_zero_positions = positions_array[positions_array != 0].reshape(-1, 3)
        if len(non_zero_positions) == 0:
            return None
        average_position = np.median(non_zero_positions, axis=0)
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

    def getMarker_target(self, _ns, _id, pos, ori):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
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
        marker.pose.position = pos
        marker.pose.orientation = ori
        return marker

    def getMarker_ego(self, _ns, _id, pos, ori):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = _ns
        marker.id = _id
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.position = pos
        marker.pose.orientation = ori
        return marker

    def node_marker_cb(self, msg):
        if self.initial_pos is None:
            return
        new_node_pts = []
        for pt in msg.points:
            x = pt.x + self.initial_pos[1]
            y = pt.y - self.initial_pos[0]
            z = 1.5
            new_node_pts.append(Point(x,y,z))
        marker = Marker()
        marker.type = Marker.SPHERE_LIST
        marker.header.frame_id = 'world'
        marker.ns = 'anchor'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale = Vector3(1,0,0)
        marker.color = ColorRGBA(1,1,0,1)
        marker.points = new_node_pts
        self.large_node_marker_pub.publish(marker)

    def points1_cb(self, msg):
        modified_points = []
        for point in pc2.read_points(msg, field_names=None, skip_nans=True):
            point_list = list(point)
            point_list[2] = -point_list[2]
            point_list[0] = -point_list[0]
            modified_points.append(point_list)
        header = msg.header
        header.frame_id = 'local'
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
        header.frame_id = 'local'
        new_msg = pc2.create_cloud(header, msg.fields, modified_points)
        self.transform_points2_pub.publish(new_msg)

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.calc_world_position()
            rate.sleep()

if __name__ == '__main__':
    try:
        uwb_localization = UWBLocalization()
        uwb_localization.execute()
    except rospy.ROSInterruptException:
        pass