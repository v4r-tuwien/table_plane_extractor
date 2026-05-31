#!/usr/bin/env python3
from open3d_ros_helper import open3d_ros_helper as orh
import rclpy
from rclpy.node import Node
import tf2_ros
import copy
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import o3d_bb_list_to_ros_bb_arr
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
from table_plane_extractor.srv import GetBBOfObjectsOnTable, GetPCOfObjectsOnTable
from extractor_of_table_planes import extract_table_planes_from_pcd
from extractor_of_table_objects import extract_objects_from_tableplane

class TableObjectsExtractorServer(Node):

    def __init__(self):
        '''
        Starts table_objects_extractor services
        Topics: /table_objects_extractor/get_bounding_boxes
                /table_objects_extractor/get_point_clouds
        '''
        super().__init__('table_objects_extractor_server')
        self.s_bb = self.create_service(GetBBOfObjectsOnTable, '/table_objects_extractor/get_bounding_boxes', self.get_object_bbs)
        self.s_pc = self.create_service(GetPCOfObjectsOnTable, '/table_objects_extractor/get_point_clouds', self.get_object_pcs)
        self.tf_wrapper = TF2Wrapper(self.node)

        
       

    def table_objects_extractor(self, pcd):
        '''
        Returns bounding boxes and pointclouds of objects found on a table plane.
        Output: list[open3d.geometry.OrientedBoundingBox] bb_arr
                list[open3d.geometry.PointCloud] pc_arr
                np.array label_img (flattened image with shape (width*height))
        '''
        
        object_params = {k: v.value for k, v in self.get_parameters_by_prefix("table_objects_extractor").items()}
        table_params = {k: v.value for k, v in self.get_parameters_by_prefix("table_plane_extractor").items()}
        
        if table_params["enable_rviz_visualization"]:
            rviz_vis = RvizVisualizer('TablePlaneExtractorVisualizer')   

        height = pcd.height
        width = pcd.width
        
        #make sure pointcloud has z pointing up
        pcd = self.tf_wrapper.transformPointCloud(pcd, table_params['base_frame'], pcd.header.frame_id) 

        header = pcd.header

        pcd_with_nans = orh.rospc_to_o3dpc(pcd, remove_nans=False)
        pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

        # downsample cloud
        pcd_downsampled = pcd.voxel_down_sample(
            voxel_size=table_params["downsample_vox_size"])

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
            return None, None, None

        bb_arr, pc_arr, label_img = extract_objects_from_tableplane(
            pcd_with_nans, 
            copy.deepcopy(bboxes),
            object_params["cluster_dbscan_eps"], 
            object_params["min_points"], 
            object_params["min_volume"], 
            object_params["max_obj_height"],
            height,
            width)
        
        if table_params["enable_rviz_visualization"]:
            rviz_vis.publish_o3d_bb_arr(bboxes, header, "table_plane")
            rviz_vis.publish_o3d_bb_arr(bb_arr, header, "objects_on_table")

        return bb_arr, pc_arr, label_img


    def get_object_bbs(self, req, response):
        '''
        Service that returns bounding boxes of objects found on a table plane
        Topic: /objects_on_table/get_bounding_boxes
        Input: sensor_msgs/PointCloud2 scene
        Output: vision_msgs/BoundingBox3DArray detected_objects
        '''

        base_frame = self.get_parameter("/table_plane_extractor/base_frame").value
        
        o3d_bb_arr, _, _ = self.table_objects_extractor(req.point_cloud)
        ros_bb_arr = o3d_bb_list_to_ros_bb_arr(
            o3d_bb_arr, base_frame, self.get_clock().now().to_msg())
        
        response.detected_objects = ros_bb_arr
        return response


    def get_object_pcs(self, req, response):
        '''Service that returns the pointclouds of objects found on a table plane
            Topic: /objects_on_table/get_point_clouds
            Input: sensor_msgs/PointCloud2 scene
            Output: sensor_msgs/PointCloud2[] detected_objects
        '''
        base_frame = self.get_parameter("/table_plane_extractor/base_frame").value

        _, o3d_pc_arr, _ = self.table_objects_extractor(req.point_cloud)

        ros_pc_arr = []
        for pc in o3d_pc_arr:
            ros_pc = orh.o3dpc_to_rospc(pc, base_frame, self.get_clock().now().to_msg())
            ros_pc_arr.append(ros_pc)

        response.detected_objects = ros_pc_arr
        return response



def main():
    rclpy.init()
    node = TableObjectsExtractorServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()