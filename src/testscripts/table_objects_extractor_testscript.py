#!/usr/bin/env python3
import rospy
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
from grasping_pipeline_msgs.srv import GetBBOfObjectsOnTable, GetPCOfObjectsOnTable
from sensor_msgs.msg import PointCloud2

# result_type = 'bb'
result_type = 'bb'

class UseGetBBOfObjectsOnTable():
    ''' Example of using the table_objects_extractor service. '''

    def __init__(self):
        self.cloud = None
        # get point cloud
        topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        sub = rospy.Subscriber(topic, PointCloud2, self.pc_cb)
        print("waiting for pointcloud message")
        rospy.wait_for_message(topic, PointCloud2)
        print("waiting for table_objects_extractor/get_bounding_boxes service")
        rospy.wait_for_service('/table_objects_extractor/get_bounding_boxes')
        try:
            print("calling service")
            rospy.set_param('/table_objects_extractor/enable_rviz_visualization', True)
            start = rospy.get_time()
            get_obj_bb = rospy.ServiceProxy('/table_objects_extractor/get_bounding_boxes', GetBBOfObjectsOnTable)
            response = get_obj_bb(self.cloud)
            print("done in " + str(rospy.get_time() - start) + " s")
            print(f"detected {len(response.detected_objects.boxes)} objects.")

        except rospy.ServiceException as e:
            print(e)

    def pc_cb(self, data):
        self.cloud = data

class UseGetPCOfObjectsOnTable():
    ''' Example of using the table_objects_extractor service. '''

    def __init__(self):
        self.cloud = None
        # get point cloud
        topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        sub = rospy.Subscriber(topic, PointCloud2, self.pc_cb)
        print("waiting for pointcloud message")
        rospy.wait_for_message(topic, PointCloud2)
        print("waiting for table_objects_extractor/get_bounding_boxes service")
        rospy.wait_for_service('/table_objects_extractor/get_bounding_boxes')
        try:
            # use service
            print("calling service")
            get_obj_pc = rospy.ServiceProxy('/table_objects_extractor/get_point_clouds', GetPCOfObjectsOnTable)
            response = get_obj_pc(self.cloud)
            for pc in response.detected_objects:
                o3d_pc = orh.rospc_to_o3dpc(pc, True)
                o3d.visualization.draw_geometries([o3d_pc])

        except rospy.ServiceException as e:
            print(e)

    def pc_cb(self, data):
        self.cloud = data


if __name__ == "__main__":
    rospy.init_node('get_objects_on_table_client')
    if result_type == 'bb':
        u = UseGetBBOfObjectsOnTable()
    elif result_type == 'pc':
        u = UseGetPCOfObjectsOnTable()
    else:
        raise TypeError("result_type unknown: " + result_type)
