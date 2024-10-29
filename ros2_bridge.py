from rclpy.node import Node
from nav_msgs.msg import Odometry

class RobotDataManager(Node):
    def __init__(self, env, num_envs):
        super().__init__("data_publisher_manager")
        self.env = env
        self.num_envs = num_envs

        self.odom_pub = []
        for i in range(self.num_envs):
            if (self.num_envs == 1):
                self.odom_pub.append(
                    self.create_publisher(Odometry, "unitree_go2/odom", 10))
            else:
                self.odom_pub.append(
                    self.create_publisher(Odometry, f"unitree_go2_{i}/odom", 10))
    
    def publish_odom(self, base_pos, base_rot, env_idx):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        if (self.num_envs == 1):
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = f"unitree_go2_{i}/base_link"
        odom_msg.pose.pose.position.x = base_pos[0].item()
        odom_msg.pose.pose.position.y = base_pos[1].item()
        odom_msg.pose.pose.position.z = base_pos[2].item()
        odom_msg.pose.pose.orientation.x = base_rot[1].item()
        odom_msg.pose.pose.orientation.y = base_rot[2].item()
        odom_msg.pose.pose.orientation.z = base_rot[3].item()
        odom_msg.pose.pose.orientation.w = base_rot[0].item()
        self.odom_pub[env_idx].publish(odom_msg)

    def pub_ros2_data(self):
        for i in range(self.num_envs):
            self.publish_odom(self.env.env.scene["unitree_go2"].data.root_state_w[i, :3],
                              self.env.env.scene["unitree_go2"].data.root_state_w[i, 3:7], i)