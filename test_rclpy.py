import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_publisher')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 82259.7543
    msg.pose.pose.position.y = 50468.6533
    msg.pose.pose.position.z = 32.432
    msg.pose.pose.orientation.w = 1.0
    
    # Publish 5 times
    for i in range(5):
        pub.publish(msg)
        print(f"Published {i}")
        import time
        time.sleep(0.5)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
