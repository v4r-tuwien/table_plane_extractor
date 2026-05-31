#!/usr/bin/env python3
from table_plane_extractor_msgs.srv import TablePlaneExtractor
from table_plane_extractor.extractor_of_table_planes import extract_table_planes_from_pcd
from grasping_pipeline_msgs.msg import Plane
import rclpy
from rclpy.node import Node
#from open3d_ros2_helper import open3d_ros2_helper as orh
import tf2_ros
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import o3d_bb_to_ros_bb
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
from vision_msgs.msg import BoundingBox3DArray

from std_msgs.msg import Header


class TablePlaneExtractorServer(Node):
    
    def __init__(self):
        ''' 
        Starting table plane extractor server node
        Topic: /table_plane_extractor/get_planes
        '''
        super().__init__('table_plane_extractor_server',automatically_declare_parameters_from_overrides=True)
        self.s = self.create_service(TablePlaneExtractor, '/table_plane_extractor/get_planes', self.table_plane_extractor_methode)
        self.tf_wrapper = TF2Wrapper(self)

        self.table_params = {k: v.value for k, v in self.get_parameters_by_prefix("table_plane_extractor_server").items()}

        self.get_logger().info("Ready to extract planes")
      
        
    def table_plane_extractor_methode(self, req, response):
        ''' 
        Table plane extractor who takes the point cloud as input. Returns possible horizontal planes 
        with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and bounding boxes around the planes.
        Input: sensor_msgs/PointCloud2 inputCloud
        Output: table_plane_extractor/Plane[] planes, vision_msgs/BoundingBox3DArray plane_bounding_boxes
        '''

        if self.table_params['enable_rviz_visualization']:
            self.get_logger().info("Initializing RViz visualizer...")
            rviz_vis = RvizVisualizer(self,'TablePlaneExtractorVisualizer') 

        # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
        pcd = req.point_cloud

        #make sure pointcloud has z pointing up
        pcd = self.tf_wrapper.transformPointCloud(pcd, self.table_params['base_frame'], pcd.header.frame_id) 
        header = pcd.header
        pcd = self.tf_wrapper.rospc_to_o3dpc_fixed(pcd, remove_nans=True)

        # downsample cloud
        pcd = pcd.voxel_down_sample(voxel_size=self.table_params['downsample_vox_size'])

        planes, bboxes = extract_table_planes_from_pcd(
            pcd, 
            self.table_params["cluster_dbscan_eps"], 
            self.table_params["min_cluster_size"], 
            self.table_params["plane_segmentation_distance_threshold"], 
            self.table_params["max_angle_deg"], 
            self.table_params["z_min"],
            self.table_params["num_iter_ransac"],
            self.table_params["min_pre_cluster_size"])
        
        if planes is None:
            self.get_logger().error("No planes found!")
            return response
        
        planes_ros = [
            Plane(
        header=Header(
            stamp=header.stamp,
            frame_id=self.table_params['base_frame']
        ),
        x=a,
        y=b,
        z=c,
        d=d
    )
    for a, b, c, d in planes]

        bb_arr = BoundingBox3DArray()
        bb_arr.header = header
        bb_arr.boxes = [o3d_bb_to_ros_bb(bb_plane) for bb_plane in bboxes]

        if self.table_params['enable_rviz_visualization']:
            rviz_vis.publish_ros_bb_arr(bb_arr, "table_plane", True)
        
        response.planes = planes_ros
        response.plane_bounding_boxes = bb_arr
        self.get_logger().info(f"Extracted {len(planes)} planes.")
        return response

def main():
    rclpy.init()
    srv = TablePlaneExtractorServer()
    rclpy.spin(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()