#!/usr/bin/python3

from grasping_pipeline_msgs.srv import TablePlaneExtractor
import rospy
from sensor_msgs.msg import PointCloud2


class UseTablePlaneExtractor():
    ''' Example of using the table plane extractor service. '''

    def __init__(self):
        self.cloud = None
        # get point cloud
        topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        sub = rospy.Subscriber(topic, PointCloud2, self.pc_cb)
        print("waiting for pointcloud message")
        rospy.wait_for_message(topic, PointCloud2)
        print("waiting for table_plane_extractor service")
        rospy.wait_for_service('/table_plane_extractor/get_planes')
        try:
            # use service
            print("calling service")
            start = rospy.get_time()
            table_extractor = rospy.ServiceProxy(
                '/table_plane_extractor/get_planes', TablePlaneExtractor)
            response = table_extractor(self.cloud)
            print((str(len(response.plane_bounding_boxes.boxes))) + ' plane(s) found!')
            print("done in " + str(rospy.get_time() - start) + " s")

        except rospy.ServiceException as e:
            print(e)

    def pc_cb(self, data):
        self.cloud = data


if __name__ == "__main__":
    rospy.init_node('table_plane_extractor_client')
    u = UseTablePlaneExtractor()
