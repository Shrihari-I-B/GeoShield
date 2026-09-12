import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def _quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)

def main():
    rclpy.init()
    node = rclpy.create_node('test_publish')
    
    # Init pose
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 82259.7543
    msg.pose.pose.position.y = 50468.6533
    msg.pose.pose.position.z = 32.432
    z, w = _quat(-2.5308672804987458)
    msg.pose.pose.orientation.z = z
    msg.pose.pose.orientation.w = w
    msg.pose.covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.06853, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.06853, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853
    ]
    
    # Goal
    pub_goal = node.create_publisher(PoseStamped, '/planning/mission_planning/goal', 1)
    msg_g = PoseStamped()
    msg_g.header.frame_id = 'map'
    msg_g.pose.position.x = 81857.4862
    msg_g.pose.position.y = 50439.2168
    msg_g.pose.position.z = 32.508
    zg, wg = _quat(-2.954200637870068)
    msg_g.pose.orientation.z = zg
    msg_g.pose.orientation.w = wg
    
    time.sleep(1.0)
    for i in range(3):
        pub_pose.publish(msg)
        time.sleep(0.5)
        pub_goal.publish(msg_g)
        time.sleep(0.5)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
