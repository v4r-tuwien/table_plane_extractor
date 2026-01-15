# table_plane_extractor
- Service for horizontal table plane extraction.  
- Services for getting objects (or multi-objects blobs if they are too close to each other) that are placed on the table plane.

## Dependencies ##
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) 
- [requirements.txt](requirements.txt)
- [package.xml](package.xml)
- [CMakeLists.txt](CMakeLists.txt)
- [grasping_pipeline_msgs](https://github.com/v4r-tuwien/grasping_pipeline_msgs)

## Services

### TablePlaneExtractor
Table plane extractor who takes the point cloud as input. Returns possible horizontal planes with plane equation ( $x, y, z, d \rightarrow a x + b y + c z + d = 0$ ) and bounding boxes around the planes.

**Service topic:** 
```
/table_plane_extractor/get_planes
```
**Input/Output:** 
```
sensor_msgs/PointCloud2 pointcloud
---
table_plane_extractor/Plane[] planes, 
vision_msgs/BoundingBox3DArray plane_bounding_boxes
```
First element of *planes* corresponds with the first element of *plane_bounding_boxes.boxes* and so on.

**File:**
```
src/table_plane_extractor_srv.py
```

### GetBBOfObjectsOnTable
Service that returns bounding boxes of objects found on a table plane. 

**Service topic:** 
```
/table_objects_extractor/get_bounding_boxes
```
**Input/Output:**
```
sensor_msgs/PointCloud2 scene_pointcloud
---
vision_msgs/BoundingBox3DArray detected_objects
```
**File:**
```
src/table_objects_extractor_as.py
```

### GetPCOfObjectsOnTable
Service that returns the point clouds of objects found on a table plane. 

**Service topic:** 
```
/table_objects_extractor/get_point_clouds
```
**Input/Output:**
```
sensor_msgs/PointCloud2 scene_pointcloud
---
sensor_msgs/PointCloud2[] detected_objects
```
**File:**
```
src/table_objects_extractor_as.py
```

### GetObjectsOnTableAS
Action server that returns a LabelImage for objects found on the table plane

**Service topic:** 
```
/table_objects_extractor/get_label_image
```

**Input/Output:**
robokudo_msgs/action/GenericImgProcAnnotator

```
sensor_msgs/Image rgb, 
sensor_msgs/Image depth
---
int32[] class_ids, 
string[] class_names, 
sensor_msgs/Image image
```

**File:**
```
src/table_objects_extractor_as.py
```

## Startup

You can start the TablePlaneExtractor service with
```
roslaunch table_plane_extractor table_plane_extractor.launch
```
You can start the GetXXOfObjectsOnTable services with
```
roslaunch table_plane_extractor table_objects_extractor.launch
```
You can start the GetObjectsOnTableAS services with
```
roslaunch table_plane_extractor table_objects_extractor_as.launch
```

## Demo

You can find demo codes in the Files, provide also good starting point/template
```
src/testscripts/table_plane_extractor_testscript.py
src/testscripts/table_objects_extractor_testscript.py
src/testscripts/table_objects_extractor_as_testscript.py
```

## Visualization
rviz visualization of plane and object detection can be found on the topic
```
TablePlaneExtractorVisualizer MarkerArray
```

rviz visualization of Label Image can be found on the topic
```
objectsOnTableLabelImage Image
```

## Status
stable, tested on Ubuntu 20.04 and ROS noetic.  

### Known Issues

## Message

### Plane.msg

#### Fields
```
Header header
float32 x
float32 y
float32 z
float32 d
```

