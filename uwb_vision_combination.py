import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import numpy as np

class UWBVisionCombination:
    def __init__(self):
        rospy.init_node('uwb_vision_combination', anonymous=True)
        
        self.threshold = 0.3
        
        self.prev_uwb_pos = None
        self.prev_update_uwb_pos = None
        self.uwb_cnt = 0
        self.curr_uwb_orient = Quaternion()
        self.curr_stamp = rospy.Time.now()
        self.prev_vision_pos = None
        self.path = Path()
        self.cnt = 0
        
        rospy.Subscriber('/uwb_odom', Odometry, self.uwb_cb)
        rospy.Subscriber('/Odometry', Odometry, self.vision_cb)
        self.corrected_uwb_pub = rospy.Publisher('/correct_uwb_odom', Odometry, queue_size=1)
        self.corrected_uwb_marker_pub = rospy.Publisher("/corrected_uwb_marker", Marker, queue_size=1)
        self.corrected_uwb_path_pub = rospy.Publisher("/corrected_uwb_path", Path, queue_size=1)

    def uwb_cb(self, data):
        current_uwb_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.curr_uwb_orient = data.pose.pose.orientation
        self.curr_stamp = data.header.stamp
        self.cnt += 1 
        
        if self.prev_uwb_pos is None:
            self.prev_uwb_pos = current_uwb_pos
            return
        if self.cnt < 5 :
            return    
        uwb_change = current_uwb_pos - self.prev_uwb_pos
        
        self.correct_position(current_uwb_pos, uwb_change)
        self.prev_uwb_pos = current_uwb_pos
        

    def vision_cb(self, data):
        current_vision_pos = np.array([data.pose.pose.position.y, -data.pose.pose.position.x, data.pose.pose.position.z])
        
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
        if  np.linalg.norm(uwb_change) > self.threshold:
            if self.prev_update_uwb_pos is not None:
                corrected_position = self.prev_update_uwb_pos - self.vision_change/3.7
                print("corrected 1 : ", current_uwb_pos, "->", self.prev_update_uwb_pos, "-", self.vision_change, "=", corrected_position)
            else:
                corrected_position = self.prev_uwb_pos - self.vision_change/3.7

                print("corrected 2 : ", current_uwb_pos, "->", self.prev_uwb_pos, "-", self.vision_change, "=", corrected_position)
            
            self.publish_corrected_position(corrected_position)
            self.prev_update_uwb_pos = corrected_position

        else:
            if self.prev_update_uwb_pos is not None:
                if np.linalg.norm(current_uwb_pos-self.prev_update_uwb_pos) > self.threshold:
                    corrected_position = self.prev_update_uwb_pos - self.vision_change/3.7
                    print("corrected 3: ", current_uwb_pos, "->", self.prev_update_uwb_pos, "-", self.vision_change, "=", corrected_position)
            
                    self.publish_corrected_position(corrected_position)
                    self.prev_update_uwb_pos = corrected_position
                else:
                    print("no change: ", self.prev_update_uwb_pos)
                    self.publish_corrected_position(self.prev_update_uwb_pos)
                    self.prev_update_uwb_pos = None
            else:
                print("no change: ", current_uwb_pos)
                self.publish_corrected_position(current_uwb_pos)
                self.prev_update_uwb_pos = None

    
    def publish_corrected_position(self, position):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.curr_stamp
        odom_msg.header.frame_id = 'world'        
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation = self.curr_uwb_orient
        self.corrected_uwb_pub.publish(odom_msg)

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.curr_stamp
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation = self.curr_uwb_orient
        self.path.header.frame_id = "world"
        self.path.header.stamp = self.curr_stamp
        self.path.poses.append(pose)

        self.corrected_uwb_path_pub.publish(self.path)

        self.uwb_cnt += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    uwb_vision_combination = UWBVisionCombination()
    uwb_vision_combination.run()
