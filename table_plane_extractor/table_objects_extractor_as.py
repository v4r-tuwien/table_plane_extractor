#! /usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.action import ActionServer
import rclpy.qos
import copy
import tf2_ros
from robokudo_msgs.action import GenericImgProcAnnotator
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from v4r_util.conversions import convert_ros_depth_img_to_pcd, convert_np_label_img_to_ros_color_img
from v4r_util.message_checks import check_for_rgb_depth
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
from open3d_ros_helper import open3d_ros_helper as orh
from v4r_util.tf2 import TF2Wrapper
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane


class GetObjectsOnTableAS(Node):

    def __init__(self):
        super().__init__('table_objects_extractor_as')
        self.tf_wrapper = TF2Wrapper()
        self.bridge = CvBridge()
        self.server = ActionServer(
            self,
            GenericImgProcAnnotator,
            '/object_detector/table_objects_extractor',
            self.execute_callback)
        self.declare_parameter('/table_plane_extractor/enable_rviz_visualization', True)
        self.enable_rviz_visualization = self.get_parameter(
            '/table_plane_extractor/enable_rviz_visualization').value
        self.declare_parameter('/table_objects_extractor_as/cam_info_topic', '/camera/camera_info')
        self.cam_info_topic = self.get_parameter('/table_objects_extractor_as/cam_info_topic').value
        self.cam_info = None
        self.cam_info_sub = self.create_subscription(
            CameraInfo, 
            self.cam_info_topic, 
            self.cam_info_callback, 
            10)
        if self.enable_rviz_visualization:
            self.visualizer = RvizVisualizer(
                'TableObjectsExtractorVisualizer')
            self.pub = self.create_publisher(
                'objectsOnTableLabelImage',
                Image, 
                rclpy.qos.QoSProfile(depth=10))
        self.get_logger().info("TableObjectsExtractor: Actionserver started")

    def cam_info_callback(self, msg):
        self.cam_info = msg

    def execute_callback(self, goal_handle):
        '''
        Actionserver that implements the GenericImgProcAnnotator message.

        The table-plane based object detector is not able to assign object 
        classes to the detected objects. This means that all objects are in 
        essence 'unknown'. This is encoded by assigning class_id = -1 and 
        class_name = 'Unknown'. Additionally the label image is returned,
        which defines which pixel from the depth image belongs to which object.

        Topic: /table_objects_extractor/get_label_image
        Expected Input: sensor_msgs/Image rgb, sensor_msgs/Image depth
        Returns: int32[] class_ids, string[] class_names, sensor_msgs/Image image
        '''
        goal = goal_handle.request
        goal_ok = check_for_rgb_depth(goal)
        if not goal_ok:
            self.get_logger().error('Not every expected message field was passed to GetObjectsOnTableAS')
            goal_handle.abort()
            return GenericImgProcAnnotator.Result()
        
        # Wait for camera info
        timeout_cnt = 0
        while self.cam_info is None and timeout_cnt < 50:  # 5 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_cnt += 1
            
        if self.cam_info is None:
            self.get_logger().error("No camera info received")
            goal_handle.abort()
            return GenericImgProcAnnotator.Result()

        table_params = {k: v.value for k, v in self.get_parameters_by_prefix("table_plane_extractor").items()}
        object_params = {k: v.value for k, v in self.get_parameters_by_prefix("table_objects_extractor").items()}

        # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
        pcd, _ = convert_ros_depth_img_to_pcd(
            goal.depth, 
            self.cam_info, 
            project_valid_depth_only=False)
        height = pcd.height
        width = pcd.width
        
        #make sure pointcloud has z pointing up
        pcd = self.tf_wrapper.transformPointCloud(pcd, table_params['base_frame'], pcd.header.frame_id) 
        header = pcd.header
        pcd_with_nans = orh.rospc_to_o3dpc(pcd, remove_nans=False)
        pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

        # downsample cloud
        pcd_downsampled = pcd.voxel_down_sample(
            voxel_size=table_params['downsample_vox_size'])

        _, bboxes = extract_table_planes_from_pcd(
            pcd_downsampled, 
            table_params["cluster_dbscan_eps"],
            table_params["min_cluster_size"],
            table_params["plane_segmentation_distance_threshold"],
            table_params["max_angle_deg"],
            table_params["z_min"],
            table_params["num_iter_ransac"],
            table_params["min_pre_cluster_size"])    
        
        if bboxes is None:
            self.get_logger().error("No planes found!")
            goal_handle.abort()
            return GenericImgProcAnnotator.Result()
        
        self.get_logger().info('Found {} planes'.format(len(bboxes)))

        bb_arr, _, labels = extract_objects_from_tableplane(
            pcd_with_nans, 
            copy.deepcopy(bboxes), 
            object_params["cluster_dbscan_eps"],
            object_params["min_points"],
            object_params["min_volume"],
            object_params["max_obj_height"],
            height,
            width)


        if labels is None:
            self.get_logger().error("No objects extracted!")
            goal_handle.abort()
            return GenericImgProcAnnotator.Result()
        
        np_label_img = labels.reshape(goal.depth.height, goal.depth.width)

        if self.enable_rviz_visualization:
            self.visualizer.publish_o3d_bb_arr(
                bboxes, 
                header, 
                "table_planes")
            self.visualizer.publish_o3d_bb_arr(
                bb_arr, 
                header, 
                "objects_on_table")
            np_rgb_img = self.bridge.cv2_to_imgmsg(goal.rgb)
            color_img = convert_np_label_img_to_ros_color_img(
                np_label_img, 
                np_rgb_img)
            self.pub.publish(color_img)

        res = GenericImgProcAnnotator.Result()
        unique_labels = np.unique(labels)
        detected_object_count = unique_labels[unique_labels != -1].shape[0]
        res.class_ids = [-1] * detected_object_count
        res.class_names = ['Unknown'] * detected_object_count
        res.pose_results = [Pose()] * detected_object_count
        res.image = self.bridge.cv2_to_imgmsg(np_label_img, encoding='16SC1')

        goal_handle.succeed()
        return res

def main():
    rclpy.init()
    node = GetObjectsOnTableAS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()