#!/usr/bin/env python3

# base
import os
import numpy as np
import cv2
# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool, Float32
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
# NuScenes
from nuscenes.nuscenes import NuScenes

class Nuscenes_Node(Node):
    def __init__(self, name="nuscenes_node"):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Starting Nuscenes Visualization Node.")

        self.read_params()

        # Most Publishers will be initialized right before the first published data
        self._publishers = {}
        self._publishers["bboxes"] = self.create_publisher(MarkerArray, "/nuscenes/bboxes",qos_profile=1)

        # NuScenes Init
        self.nusc = NuScenes(version=self.nuscenes_version, 
                             dataroot=self.nuscenes_dir, 
                             verbose=True)
        print(len(self.nusc.scene))
        
        # Initialize for control status
        self.set_index(1)
        self.pause = False
        self.stop = True
        self.publishing = True
        self.timer = self.create_timer(1.0 / self.update_frequency, self.publish_callback)

    def read_params(self):
        self.nuscenes_dir = self.get_parameter_or(
            '~NUSCENES_DIR', rclpy.Parameter('~NUSCENES_DIR', rclpy.Parameter.Type.STRING, '/home/antonio/Data/nuscenes/Full_dataset_v1.0/mini/')).value
        self.nuscenes_version = self.get_parameter_or(
            '~NUSCENES_VER', rclpy.Parameter('~NUSCENES_VER', rclpy.Parameter.Type.STRING, 'v1.0-mini')).value
        self.update_frequency = self.get_parameter_or(
            "~UPDATE_FREQUENCY", rclpy.Parameter('~UPDATE_FREQUENCY', rclpy.Parameter.Type.INTEGER, 8)).value

        self.get_logger().info("dir: %s, ver: %s, freq: %s" %
                           (str(self.nuscenes_dir),
                            str(self.nuscenes_version),
                            str(self.update_frequency),))

    def set_index(self, index):
        """Set current index -> select scenes -> print(scene description) -> set current sample as the first sample of the scene
        """
        self.index = index
        self.current_scene = self.nusc.scene[self.index]
        des = self.current_scene["description"]
        print(f"Switch to scenes {self.index}: {des} ")
        self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token'])
    
    def _camera_publish(self, camera_data, is_publish_image=False):
        """Publish camera related data, first publish pose/tf information, then publish image if needed

        Args:
            camera_data (Dict): camera data from nuscenes' sample data
            is_publish_image (bool, optional): determine whether to read image data. Defaults to False.
        """        
        cs_record   = self.nusc.get("calibrated_sensor", camera_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", camera_data['ego_pose_token'])

        image_path = os.path.join(self.nuscenes_dir, camera_data['filename'])
        imsize    = (camera_data["width"], camera_data["height"])
        channel = camera_data['channel']
        
        # publish relative pose
        cam_intrinsic = np.array(cs_record["camera_intrinsic"]) #[3 * 3]
        rotation = cs_record["rotation"] #list, [4] r, x, y, z
        translation = cs_record['translation'] #
        # relative_pose = ros_util.compute_pose(translation, rotation)
        pose_pub_name = f"{channel}_pose_pub"
        # if pose_pub_name not in self._publishers:
        #     self.publishers[pose_pub_name] = self.create_publisher(f"/nuscenes/{channel}/pose", PoseStamped, queue_size=1, latch=True)
        # msg = PoseStamped()
        # msg.pose = relative_pose
        # msg.header.frame_id = "base_link"
        # msg.header.stamp = rospy.Time.now()
        # self.publishers[pose_pub_name].publish(msg)

        if is_publish_image:
            image_pub_name = f"{channel}_image_pub"
            if image_pub_name not in self._publishers:
                self._publishers[image_pub_name] = self.create_publisher(Image, f"/nuscenes/{channel}/image", qos_profile=10)
            info_pub_name  = f"{channel}_info_pub"
            if info_pub_name not in self._publishers:
                self._publishers[info_pub_name] = self.create_publisher(CameraInfo, f"/nuscenes/{channel}/camera_info", qos_profile=10)

            image = cv2.imread(image_path)
            self._publishers[image_pub_name].publish(CvBridge().cv2_to_imgmsg(image))
            # self._publishers[info_pub_name].publish()
            # ros_util.publish_image(image,
            #                        self.publishers[image_pub_name],
            #                        self.publishers[info_pub_name],
            #                        cam_intrinsic,
            #                        channel)
    def _lidar_publish(self, lidar_data, is_publish_lidar=False):
        """Publish lidar related data, first publish pose/tf information and ego pose, then publish lidar if needed

        Args:
            lidar_data (Dict): lidar data from nuscenes' sample data
            is_publish_lidar (bool, optional): determine whether to read lidar data. Defaults to False.
        """
        cs_record   = self.nusc.get("calibrated_sensor", lidar_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", lidar_data['ego_pose_token'])

        lidar_path = os.path.join(self.nuscenes_dir, lidar_data['filename'])
        channel = 'LIDAR_TOP'

    def publish_callback(self, event=None):
        self.get_logger().info("node loop")

        # Publish cameras and camera info
        channels = ['CAM_BACK',  'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT', 'CAM_BACK_LEFT']
        for channel in channels:
            data = self.nusc.get('sample_data', self.current_sample['data'][channel])
            self._camera_publish(data, is_publish_image=self.publishing)

        if True:
            if (self.current_sample['next'] == ''):
                # If end reached, loop back from the start
                self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token'])
            else:
                self.current_sample = self.nusc.get('sample', self.current_sample['next'])

def main(args=None):
    rclpy.init(args=args)                           # 1.初始化rclpy
    node = Nuscenes_Node("nuscenes_Node")           # 2.创建一个节点，名为“talker”
    node.get_logger().info("in main, spin begin")   # 3.打印输出日志信息
    rclpy.spin(node)                                # 4.运行节点
    rclpy.shutdown()                                # 5.关闭rclpy

if __name__ == "__main__":
    main()
    