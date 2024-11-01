from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import go2_ctrl

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)


class RobotDataManager(Node):
    def __init__(self, env, lidar_annotators, cameras):
        super().__init__("robot_data_manager")
        self.env = env
        self.num_envs = env.unwrapped.scene.num_envs
        self.lidar_annotators = lidar_annotators
        self.cameras = cameras
        self.points = []

        # ROS2 Broadcaster
        self.broadcaster= TransformBroadcaster(self)
        
        # ROS2 Publisher
        self.odom_pub = []
        self.pose_pub = []
        self.lidar_pub = []
        self.color_img_pub = []
        self.depth_img_pub = []
        self.cam_depth_cloud_pub = []

        # ROS2 Subscriber
        self.cmd_vel_sub = []
        self.color_img_sub = []
        self.depth_img_sub = []
        self.color_img_sub = []
        self.depth_img_sub = []
        self.cam_depth_cloud_sub = []

        # ROS2 Timer
        self.lidar_publish_timer = []
        for i in range(self.num_envs):
            if (self.num_envs == 1):
                self.odom_pub.append(
                    self.create_publisher(Odometry, "unitree_go2/odom", 10))
                self.pose_pub.append(
                    self.create_publisher(PoseStamped, "unitree_go2/pose", 10))
                self.lidar_pub.append(
                    self.create_publisher(PointCloud2, "unitree_go2/lidar/point_cloud", 10)
                )
                self.color_img_pub.append(
                    self.create_publisher(Image, "unitree_go2/front_cam/color_image", 10)
                )
                self.depth_img_pub.append(
                    self.create_publisher(Image, "unitree_go2/front_cam/depth_image", 10)
                )
                self.cam_depth_cloud_pub.append(
                    self.create_publisher(PointCloud2, "unitree_go2/front_cam/depth_cloud", 10)
                )
                self.cmd_vel_sub.append(
                    self.create_subscription(Twist, "unitree_go2/cmd_vel", 
                    lambda msg: self.cmd_vel_callback(msg, 0), 10)
                )
                self.color_img_sub.append(
                    self.create_subscription(Image, "unitree_go2/front_cam/color_image_raw", 
                    lambda msg: self.color_image_callback(msg, 0), 10)
                )
                self.depth_img_sub.append(
                    self.create_subscription(Image, "unitree_go2/front_cam/depth_image_raw", 
                    lambda msg: self.depth_image_callback(msg, 0), 10)
                )
                self.cam_depth_cloud_sub.append(
                    self.create_subscription(PointCloud2, "unitree_go2/front_cam/depth_cloud_raw", 
                    lambda msg: self.cam_depth_cloud_callback(msg, 0), 10)
                )
            else:
                self.odom_pub.append(
                    self.create_publisher(Odometry, f"unitree_go2_{i}/odom", 10))
                self.pose_pub.append(
                    self.create_publisher(PoseStamped, f"unitree_go2_{i}/pose", 10))
                self.lidar_pub.append(
                    self.create_publisher(PointCloud2, f"unitree_go2_{i}/lidar/point_cloud", 10)
                )
                self.color_img_pub.append(
                    self.create_publisher(Image, f"unitree_go2_{i}/front_cam/color_image", 10)
                )
                self.depth_img_pub.append(
                    self.create_publisher(Image, f"unitree_go2_{i}/front_cam/depth_image", 10)
                )
                self.cam_depth_cloud_pub.append(
                    self.create_publisher(PointCloud2, f"unitree_go2_{i}/front_cam/depth_cloud", 10)
                )
                self.cmd_vel_sub.append(
                    self.create_subscription(Twist, f"unitree_go2_{i}/cmd_vel", 
                    lambda msg, env_idx=i: self.cmd_vel_callback(msg, env_idx), 10)
                )
                self.color_img_sub.append(
                    self.create_subscription(Image, "unitree_go2/front_cam/color_image_raw", 
                    lambda msg, env_idx=i: self.color_image_callback(msg, env_idx), 10)
                )
                self.depth_img_sub.append(
                    self.create_subscription(Image, "unitree_go2/front_cam/depth_image_raw", 
                    lambda msg, env_idx=i: self.depth_image_callback(msg, env_idx), 10)
                )
                self.cam_depth_cloud_sub.append(
                    self.create_subscription(PointCloud2, f"unitree_go2_{i}/front_cam/depth_cloud_raw", 
                    lambda msg, env_idx=i: self.cam_depth_cloud_callback(msg, env_idx), 10)
                )
        
        self.create_timer(0.033, self.pub_ros2_data_callback)
        self.create_timer(0.1, self.pub_lidar_data_callback)
        self.create_static_transform()
        self.create_camera_publisher()
        

    def create_static_transform(self):
        for i in range(self.num_envs):
            # LiDAR
            # Create and publish the transform
            lidar_broadcaster = StaticTransformBroadcaster(self)
            base_lidar_transform = TransformStamped()
            base_lidar_transform.header.stamp = self.get_clock().now().to_msg()
            if (self.num_envs == 1):
                base_lidar_transform.header.frame_id = "unitree_go2/base_link"
                base_lidar_transform.child_frame_id = "unitree_go2/lidar_frame"
            else:
                base_lidar_transform.header.frame_id = f"unitree_go2_{i}/base_link"
                base_lidar_transform.child_frame_id = f"unitree_go2_{i}/lidar_frame"

            # Translation
            base_lidar_transform.transform.translation.x = 0.2
            base_lidar_transform.transform.translation.y = 0.0
            base_lidar_transform.transform.translation.z = 0.2
            
            # Rotation 
            base_lidar_transform.transform.rotation.x = 0.0
            base_lidar_transform.transform.rotation.y = 0.0
            base_lidar_transform.transform.rotation.z = 0.0
            base_lidar_transform.transform.rotation.w = 1.0
            
            # Publish the transform
            lidar_broadcaster.sendTransform(base_lidar_transform)
    
            # -------------------------------------------------------------
            # Camera
            # Create and publish the transform
            camera_broadcaster = StaticTransformBroadcaster(self)
            base_cam_transform = TransformStamped()
            base_cam_transform.header.stamp = self.get_clock().now().to_msg()
            if (self.num_envs == 1):
                base_cam_transform.header.frame_id = "unitree_go2/base_link"
                base_cam_transform.child_frame_id = "unitree_go2/front_cam"
            else:
                base_cam_transform.header.frame_id = f"unitree_go2_{i}/base_link"
                base_cam_transform.child_frame_id = f"unitree_go2_{i}/front_cam"

            # Translation
            base_cam_transform.transform.translation.x = 0.4
            base_cam_transform.transform.translation.y = 0.0
            base_cam_transform.transform.translation.z = 0.2
            
            # Rotation 
            base_cam_transform.transform.rotation.x = -0.5
            base_cam_transform.transform.rotation.y = 0.5
            base_cam_transform.transform.rotation.z = -0.5
            base_cam_transform.transform.rotation.w = 0.5
            
            # Publish the transform
            camera_broadcaster.sendTransform(base_cam_transform)
    
    def create_camera_publisher(self):
        self.pub_color_image()
        self.pub_depth_image()
        self.pub_cam_depth_cloud()
    
    def publish_odom(self, base_pos, base_rot, base_lin_vel_b, base_ang_vel_b, env_idx):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        if (self.num_envs == 1):
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = f"unitree_go2_{env_idx}/base_link"
        odom_msg.pose.pose.position.x = base_pos[0].item()
        odom_msg.pose.pose.position.y = base_pos[1].item()
        odom_msg.pose.pose.position.z = base_pos[2].item()
        odom_msg.pose.pose.orientation.x = base_rot[1].item()
        odom_msg.pose.pose.orientation.y = base_rot[2].item()
        odom_msg.pose.pose.orientation.z = base_rot[3].item()
        odom_msg.pose.pose.orientation.w = base_rot[0].item()
        odom_msg.twist.twist.linear.x = base_lin_vel_b[0].item()
        odom_msg.twist.twist.linear.y = base_lin_vel_b[1].item()
        odom_msg.twist.twist.linear.z = base_lin_vel_b[2].item()
        odom_msg.twist.twist.angular.x = base_ang_vel_b[0].item()
        odom_msg.twist.twist.angular.y = base_ang_vel_b[1].item()
        odom_msg.twist.twist.angular.z = base_ang_vel_b[2].item()
        self.odom_pub[env_idx].publish(odom_msg)

        # transform
        map_base_trans = TransformStamped()
        map_base_trans.header.stamp = self.get_clock().now().to_msg()
        map_base_trans.header.frame_id = "map"
        if (self.num_envs == 1):
            map_base_trans.child_frame_id = "unitree_go2/base_link"
        else:
            map_base_trans.child_frame_id = f"unitree_go2_{env_idx}/base_link"
        map_base_trans.transform.translation.x = base_pos[0].item()
        map_base_trans.transform.translation.y = base_pos[1].item()
        map_base_trans.transform.translation.z = base_pos[2].item()
        map_base_trans.transform.rotation.x = base_rot[1].item()
        map_base_trans.transform.rotation.y = base_rot[2].item()
        map_base_trans.transform.rotation.z = base_rot[3].item()
        map_base_trans.transform.rotation.w = base_rot[0].item()
        self.broadcaster.sendTransform(map_base_trans)
    
    def publish_pose(self, base_pos, base_rot, env_idx):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = base_pos[0].item()
        pose_msg.pose.position.y = base_pos[1].item()
        pose_msg.pose.position.z = base_pos[2].item()
        pose_msg.pose.orientation.x = base_rot[1].item()
        pose_msg.pose.orientation.y = base_rot[2].item()
        pose_msg.pose.orientation.z = base_rot[3].item()
        pose_msg.pose.orientation.w = base_rot[0].item()
        self.pose_pub[env_idx].publish(pose_msg)

    def publish_lidar_data(self, points, env_idx):
        point_cloud = PointCloud2()
        if (self.num_envs == 1):
            point_cloud.header.frame_id = "unitree_go2/lidar_frame"
        else:
            point_cloud.header.frame_id = f"unitree_go2_{env_idx}/lidar_frame"
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
        self.lidar_pub[env_idx].publish(point_cloud)        

    def pub_ros2_data_callback(self):
        robot_data = self.env.unwrapped.scene["unitree_go2"].data
        for i in range(self.num_envs):
            self.publish_odom(robot_data.root_state_w[i, :3],
                              robot_data.root_state_w[i, 3:7],
                              robot_data.root_lin_vel_b[i],
                              robot_data.root_ang_vel_b[i],
                              i)
            self.publish_pose(robot_data.root_state_w[i, :3],
                              robot_data.root_state_w[i, 3:7], i)

    def pub_lidar_data_callback(self):
        for i in range(self.num_envs):
            self.publish_lidar_data(self.lidar_annotators[i].get_data()["data"].reshape(-1, 3), i)

    def cmd_vel_callback(self, msg, env_idx):
        go2_ctrl.base_vel_cmd_input[env_idx][0] = msg.linear.x
        go2_ctrl.base_vel_cmd_input[env_idx][1] = msg.linear.y
        go2_ctrl.base_vel_cmd_input[env_idx][2] = msg.angular.z

    def color_image_callback(self, msg, env_idx):
        msg.header.stamp = self.get_clock().now().to_msg()
        if (self.num_envs == 1):
            msg.header.frame_id = "unitree_go2/front_cam"
        else:
            msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"     
        self.color_img_pub[env_idx].publish(msg)   
        # # Update timestamp and frame ID in header
        # new_msg = Image()
        # new_msg.header.stamp = self.get_clock().now().to_msg()
        # if (self.num_envs == 1):
        #     new_msg.header.frame_id = "unitree_go2/front_cam"
        # else:
        #     new_msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"

        # # Copy other message fields
        # new_msg.height = msg.height
        # new_msg.width = msg.width
        # new_msg.encoding = msg.encoding
        # new_msg.is_bigendian = msg.is_bigendian
        # new_msg.step = msg.step
        # new_msg.data = msg.data
        # self.color_img_pub[env_idx].publish(new_msg)

    def depth_image_callback(self, msg, env_idx):
        msg.header.stamp = self.get_clock().now().to_msg()
        if (self.num_envs == 1):
            msg.header.frame_id = "unitree_go2/front_cam"
        else:
            msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"     
        self.depth_img_pub[env_idx].publish(msg)   
        # Update timestamp and frame ID in header
        # new_msg = Image()
        # new_msg.header.stamp = self.get_clock().now().to_msg()
        # if (self.num_envs == 1):
        #     new_msg.header.frame_id = "unitree_go2/front_cam"
        # else:
        #     new_msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"

        # # Copy other message fields
        # new_msg.height = msg.height
        # new_msg.width = msg.width
        # new_msg.encoding = msg.encoding
        # new_msg.is_bigendian = msg.is_bigendian
        # new_msg.step = msg.step
        # new_msg.data = msg.data
        # self.depth_img_pub[env_idx].publish(new_msg)

    def cam_depth_cloud_callback(self, msg, env_idx):
        msg.header.stamp = self.get_clock().now().to_msg()
        if (self.num_envs == 1):
            msg.header.frame_id = "unitree_go2/front_cam"
        else:
            msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"     
        self.cam_depth_cloud_pub[env_idx].publish(msg)   
        # Modify the timestamp and frame_id
        # new_msg = PointCloud2()
        # new_msg.header.stamp = self.get_clock().now().to_msg()  
        # if (self.num_envs == 1):
        #     new_msg.header.frame_id = "unitree_go2/front_cam"
        # else:
        #     new_msg.header.frame_id = f"unitree_go2_{env_idx}/front_cam"
        # # Copy the rest of the fields from the original message
        # new_msg.height = msg.height
        # new_msg.width = msg.width
        # new_msg.fields = msg.fields
        # new_msg.is_bigendian = msg.is_bigendian
        # new_msg.point_step = msg.point_step
        # new_msg.row_step = msg.row_step
        # new_msg.is_dense = msg.is_dense
        # new_msg.data = msg.data

        # # Publish the modified message
        # self.cam_depth_cloud_pub[env_idx].publish(new_msg)        

    def pub_color_image(self):
        for i in range(self.num_envs):
            # The following code will link the camera's render product and publish the data to the specified topic name.
            render_product = self.cameras[i]._render_product_path
            freq = 30
            step_size = int(60/freq)
            topic_name = "/front_cam/color_image_raw"
            if (self.num_envs == 1):
                node_namespace = "unitree_go2"                
            else:
                node_namespace = f"unitree_go2_{i}"            
            queue_size = 1
            frame_id = self.cameras[i].prim_path.split("/")[-1] + f"_color_{i}" # This matches what the TF tree is publishing.

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            
            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace=node_namespace,
                queueSize=queue_size,
                topicName=topic_name,
            )
            writer.attach([render_product])

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def pub_depth_image(self):
        for i in range(self.num_envs):
            # The following code will link the camera's render product and publish the data to the specified topic name.
            render_product = self.cameras[i]._render_product_path
            freq = 30 # TODO: 
            step_size = int(60/freq)
            topic_name = "/front_cam/depth_image_raw"
            if (self.num_envs == 1):
                node_namespace = "unitree_go2"                
            else:
                node_namespace = f"unitree_go2_{i}"
            queue_size = 1
            frame_id = self.cameras[i].prim_path.split("/")[-1] + f"_depth_{i}" # This matches what the TF tree is publishing.

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                                    sd.SensorType.DistanceToImagePlane.name
                                )
            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace=node_namespace,
                queueSize=queue_size,
                topicName=topic_name
            )
            writer.attach([render_product])

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
            

    def pub_cam_depth_cloud(self):
        for i in range(self.num_envs):
            # The following code will link the camera's render product and publish the data to the specified topic name.
            render_product = self.cameras[i]._render_product_path
            freq = 30
            step_size = int(60/freq)
            topic_name = "/front_cam/depth_cloud_raw"
            if (self.num_envs == 1):
                node_namespace = "unitree_go2"                
            else:
                node_namespace = f"unitree_go2_{i}"            
            queue_size = 1
            frame_id = self.cameras[i].prim_path.split("/")[-1] + f"_depth_cloud_{i}"  # This matches what the TF tree is publishing.

            # Note, this pointcloud publisher will simply convert the Depth image to a pointcloud using the Camera intrinsics.
            # This pointcloud generation method does not support semantic labelled objects.
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.DistanceToImagePlane.name
            )

            writer = rep.writers.get(rv + "ROS2PublishPointCloud")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace=node_namespace,
                queueSize=queue_size,
                topicName=topic_name
            )
            writer.attach([render_product])

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )

            og.Controller.attribute(gate_path + ".inputs:step").set(step_size)   