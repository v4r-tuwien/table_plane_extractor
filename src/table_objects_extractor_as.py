#! /usr/bin/env python3
import numpy as np
import rospy
import ros_numpy
import actionlib
import copy
import tf2_ros
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from v4r_util.conversions import convert_ros_depth_img_to_pcd, convert_np_label_img_to_ros_color_img
from v4r_util.message_checks import check_for_rgb_depth
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
from open3d_ros_helper import open3d_ros_helper as orh
from v4r_util.tf2 import TF2Wrapper
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane


class GetObjectsOnTableAS():

    def __init__(self):

        self.tf_wrapper = TF2Wrapper()
        self.server = actionlib.SimpleActionServer(
            '/object_detector/table_objects_extractor', 
            GenericImgProcAnnotatorAction, 
            self.get_labels_img, 
            False)

        self.enable_rviz_visualization = rospy.get_param(
            '/table_plane_extractor/enable_rviz_visualization')
        if self.enable_rviz_visualization:
            self.visualizer = RvizVisualizer(
                'TableObjectsExtractorVisualizer')
            self.pub = rospy.Publisher(
                'objectsOnTableLabelImage',
                Image, 
                queue_size=10)
        self.server.start()
        rospy.loginfo("TableObjectsExtractor: Actionserver started")

    def get_labels_img(self, goal):
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
        goal_ok = check_for_rgb_depth(goal)
        if not goal_ok:
            self.server.set_aborted(
                'Not every expected message field was passed to GetObjectsOnTableAS')
            return
        
        ros_cam_topic = rospy.get_param('/table_objects_extractor_as/cam_info_topic')
        cam_info = rospy.wait_for_message(ros_cam_topic, CameraInfo)

        table_params = rospy.get_param("/table_plane_extractor")
        object_params = rospy.get_param("/table_objects_extractor")

        # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
        pcd, _ = convert_ros_depth_img_to_pcd(
            goal.depth, 
            cam_info, 
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
            rospy.logerr("No planes found!")
            self.server.set_aborted(None)
            return
        
        print('Found {} planes'.format(len(bboxes)))

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
            rospy.logerr("No objects extracted!")
            self.server.set_aborted(None)
            return
        
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
            np_rgb_img = ros_numpy.numpify(goal.rgb)
            color_img = convert_np_label_img_to_ros_color_img(
                np_label_img, 
                np_rgb_img)
            self.pub.publish(color_img)

        res = GenericImgProcAnnotatorResult()
        unique_labels = np.unique(labels)
        detected_object_count = unique_labels[unique_labels != -1].shape[0]
        res.class_ids = [-1] * detected_object_count
        res.class_names = ['Unknown'] * detected_object_count
        res.pose_results = [Pose()] * detected_object_count
        res.image = ros_numpy.msgify(Image, np_label_img, encoding='16SC1')

        self.server.set_succeeded(res)

if __name__ == '__main__':
    rospy.init_node('table_objects_extractor_as')
    server = GetObjectsOnTableAS()
    rospy.spin()