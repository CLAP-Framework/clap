# Package Summary
This package provides object detection algorithm using lidar point cloud

# Node `euclidean_cluster`
This node using ground removal and Euclidean Clustring to generate point cloud detections

## Input Topic
|Topic|Condition|Type|Description|
|---|---|---|---|
|`points_raw`||`sensor_msgs/PointCloud2`|Input point cloud|
## Output Topic
|Topic|Condition|Type|Description|
|---|---|---|---|
|`objects_detected`||`zzz_perception_msgs/DetectionBoxArray`|Detected objects|
|`points_cluster`|Debug|`sensor_msgs/PointCloud2`|Points after clustering|
|`points_ground`|Debug|`sensor_msgs/PointCloud2`|Points of removed ground|
## Parameters
|Parameter|Type|Description|Default|
|---|---|---|---|
|`downsampling_enable`|bool|Enable downsampling pre-processing|`false`|
|`downsampling_leaf_size`|float|The size of grid used in downsampling. Lower the size is, less the points are|`0.1`|
|`crop_enable`|bool|Enable crop pre-processing|`false`|
|`plane_removal_enable`|bool|Enable planes removal pre-processing|`true`|
|`plane_removal_ransac_thres`|float|Plane RANSAC distance threshold|`0.15`|
|`plane_removal_count_thres`|float|Least proportion of the plane points to be removed|`0.1`|
|`cluster_dist_thres`|float|Euclidean Clustering distance threshold|`0.5`|
|`cluster_count_min`|integer|Minimum point count of valid cluster|`100`|
|`cluster_count_max`|integer|Maximum point count of valid cluster|`1000000`|
## Reference
1. Point Cloud Library:
  ```tex
  @InProceedings{Rusu_ICRA2011_PCL,
    author = {Radu Bogdan Rusu and Steve Cousins},
    title = {{3D is here: Point Cloud Library (PCL)}},
    booktitle = {{IEEE International Conference on Robotics and Automation (ICRA)}},
    month = {May 9-13},
    year = {2011},
    address = {Shanghai, China}
  }
  ```

1. Euclidean Clustering:
  ```tex
  @PhDThesis{RusuDoctoralDissertation,
    author = {Radu Bogdan Rusu},
    title = {Semantic 3D Object Maps for Everyday Manipulation in Human Living Environments},
    school = {Computer Science department, Technische Universitaet Muenchen, Germany},
    year = {2009},
    month = {October}
  }
  ```
