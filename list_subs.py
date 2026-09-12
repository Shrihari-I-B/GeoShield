import rclpy
import time
rclpy.init()
node = rclpy.create_node('list_subs')
time.sleep(2)  # wait for discovery
print("Topics:")
for topic_name, types in node.get_topic_names_and_types():
    print(topic_name, types)
print("Nodes:")
for n in node.get_node_names():
    print(n)
node.destroy_node()
rclpy.shutdown()
