import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_goal_pub')
    pub = node.create_publisher(PoseStamped, '/planning/mission_planning/goal', 1)
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = 81857.4862
    msg.pose.position.y = 50439.2168
    msg.pose.position.z = 32.508
    msg.pose.orientation.w = 1.0
    
    for i in range(5):
        pub.publish(msg)
        print(f"Published goal {i}")
        import time
        time.sleep(0.5)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
