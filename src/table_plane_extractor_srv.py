#!/usr/bin/python3
from grasping_pipeline_msgs.srv import TablePlaneExtractor, TablePlaneExtractorResponse
from extractor_of_table_planes import extract_table_planes_from_pcd
from grasping_pipeline_msgs.msg import Plane
import rospy
from open3d_ros_helper import open3d_ros_helper as orh
import tf2_ros
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import o3d_bb_to_ros_bb
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
from vision_msgs.msg import BoundingBox3DArray

from std_msgs.msg import Header


class TablePlaneExtractorServer():
    
    def __init__(self):
        ''' 
        Starting table plane extractor server node
        Topic: /table_plane_extractor/get_planes
        '''
        rospy.init_node('table_plane_extractor_server')
        s = rospy.Service(
            '/table_plane_extractor/get_planes',
            TablePlaneExtractor, 
            self.table_plane_extractor_methode)
        self.tf_wrapper = TF2Wrapper()
        print("Ready to extract planes")
        

    def table_plane_extractor_methode(self, req):
        ''' 
        Table plane extractor who takes the point cloud as input. Returns possible horizontal planes 
        with plane equation (x, y, z, d -> a * x + b * y + c * z + d = 0) and bounding boxes around the planes.
        Input: sensor_msgs/PointCloud2 inputCloud
        Output: table_plane_extractor/Plane[] planes, vision_msgs/BoundingBox3DArray plane_bounding_boxes
        '''

        table_params = rospy.get_param("table_plane_extractor")

        if table_params["enable_rviz_visualization"]:
            rviz_vis = RvizVisualizer('TablePlaneExtractorVisualizer') 

        # get pointcloud and convert from sensor_msgs/Pointcloud2 to open3d.geometry.PointCloud
        pcd = req.point_cloud

        #make sure pointcloud has z pointing up
        pcd = self.tf_wrapper.transformPointCloud(pcd, table_params['base_frame'], pcd.header.frame_id) 
        header = pcd.header
        pcd = orh.rospc_to_o3dpc(pcd, remove_nans=True)

        # downsample cloud
        pcd = pcd.voxel_down_sample(voxel_size=table_params['downsample_vox_size'])

        planes, bboxes = extract_table_planes_from_pcd(
            pcd, 
            table_params["cluster_dbscan_eps"], 
            table_params["min_cluster_size"], 
            table_params["plane_segmentation_distance_threshold"], 
            table_params["max_angle_deg"], 
            table_params["z_min"],
            table_params["num_iter_ransac"],
            table_params["min_pre_cluster_size"])
        
        if planes is None:
            rospy.logerr("No planes found!")
            return None, None
        
        planes_ros = [
            Plane(
                Header(0, header.stamp, table_params['base_frame']), 
                a, b, c, d) for a, b, c, d in planes]

        bb_arr = BoundingBox3DArray()
        bb_arr.header = header
        bb_arr.boxes = [o3d_bb_to_ros_bb(bb_plane) for bb_plane in bboxes]

        if table_params['enable_rviz_visualization']:
            rviz_vis.publish_ros_bb_arr(bb_arr, "table_plane", True)
        
        return TablePlaneExtractorResponse(planes_ros, bb_arr)

if __name__ == "__main__":
    srv = TablePlaneExtractorServer()
    rospy.spin()
