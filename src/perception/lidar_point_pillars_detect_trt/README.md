# Point Pillars Lidar Objects detection

This version is limited to the Xavier platform.

**Recompile autoware_msgs to build the node.**

## How to launch

* build trt model on arm platform:

```
./scripts/pointpillars_trt_builder_arm.sh
```
the trt model will be ready at `./model/pointpillars/pointpillars-1.onnx_FP16.trt`
* build trt model on x86 platform:

```
./scripts/pointpillars_trt_builder_x86.sh
```
the trt model will be ready at `./model/pointpillars/pointpillars-1-x86.onnx_FP32.trt`

Using launch file:
`roslaunch lidar_pointpillars_detect_trt lidar_pointpillars_detect_trt.launch`

* To display the results in Rviz `objects_visualizer` is required.
(Launch file launches automatically this node).