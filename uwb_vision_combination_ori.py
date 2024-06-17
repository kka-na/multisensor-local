import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker

import numpy as np

class UWBVisionCombination:
    def __init__(self):
        rospy.init_node('uwb_vision_combination', anonymous=True)
        
        self.threshold = 3
        
        self.prev_uwb_pos = None
        self.uwb_cnt = 0
        self.curr_uwb_orient = Quaternion()
        self.curr_stamp = rospy.Time.now()
        self.prev_vision_pos = None
        
        rospy.Subscriber('/uwb_odom', Odometry, self.uwb_cb)
        rospy.Subscriber('/Odometry', Odometry, self.vision_cb)
        self.corrected_uwb_pub = rospy.Publisher('/correct_uwb_odom', Odometry, queue_size=1)
        self.corrected_uwb_marker_pub = rospy.Publisher("/ciorrected_uwb_marker", Marker, queue_size=1)

    def uwb_cb(self, data):
        current_uwb_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.curr_uwb_orient = data.pose.pose.orientation
        self.curr_stamp = data.header.stamp
        if self.prev_uwb_pos is None:
            self.prev_uwb_pos = current_uwb_pos
            return
        
        uwb_change = current_uwb_pos - self.prev_uwb_pos
        self.correct_position(current_uwb_pos, uwb_change)
        self.prev_uwb_pos = current_uwb_pos
        

    def vision_cb(self, data):
        current_vision_pos = np.array([-data.pose.pose.position.x, -data.pose.pose.position.y, -data.pose.pose.position.z])
        
        if self.prev_vision_pos is None:
            self.prev_vision_pos = current_vision_pos
            return
        
        vision_change = current_vision_pos - self.prev_vision_pos

        self.prev_vision_pos = current_vision_pos
        
        self.vision_change = vision_change
    
    #TODO
    def correct_position(self, current_uwb_pos, uwb_change):
        if self.prev_vision_pos is None or not hasattr(self, 'vision_change'):
            return
        if  np.linalg.norm(uwb_change)> self.threshold:
            rospy.loginfo(f"UWB outlier detected: UWB {current_uwb_pos}, Vision change {self.vision_change}")
            corrected_position = self.prev_uwb_pos + self.vision_change
            self.publish_corrected_position(corrected_position)            
        else:
            self.publish_corrected_position(current_uwb_pos)

    
    def publish_corrected_position(self, position):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.curr_stamp
        odom_msg.header.frame_id = 'world'        
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation = self.curr_uwb_orient
        self.corrected_uwb_pub.publish(odom_msg)
        marker = self.getMarker_ego('corrected_uwb_marker', self.uwb_cnt, Point(*position), self.curr_uwb_orient)
        self.corrected_uwb_marker_pub.publish(marker)

        self.uwb_cnt += 1

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
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.position = pos
        marker.pose.orientation = ori
        return marker


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    uwb_vision_combination = UWBVisionCombination()
    uwb_vision_combination.run()
