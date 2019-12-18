This module read calibration settings from file and broadcast them by proper messages (Transform, CameraInfo, etc.)
The content of the calibration file should contain following contents

```yaml
- The whole parameters are represented among `frame`s. A `frame` is a representation of 6D pose of a sensor.

- extrinsic:
  - This field represent extrinsic parameters between frames

  - frame_id: frame name
  - child_frame_id: child frame name
  - txyz: 3-D translation from frame_id to child_frame_id
  - qxyzw: 3-D rotation from frame_id to child_frame_id in quaternion
  - valid_date: The extrinsic parameters are not always fixed, this field define which period the parameters are valid, in format 'yymmdd-yymmdd'

- intrinsic:
  - This field represent intrinsic parameters of sensors. Usually it's used to store camera intrinsics following ROS standard style

  - frame_id: frame name of the sensor
  - width: width of the image in pixels
  - height: height of the image in pixels
  - D: distortion coefficients of a camera (k1, k2, p1, p2, k3)
  - K: 3x3 intrinsic matrix of a camera. Diagonals are fx, fy, 1, last column is cx, cy, 1
  - P: 3x4 new projection matrix, convert coordinate from u,v to x,y,z
  - R: 3x3 rectification transformation of stereo cameras
```

Currently only json format is supported. Example json:
```json
{
    "extrinsics": [
        {
            "txyz": [
                -0.02518118,
                -0.14178041,
                -0.5595681
            ],
            "qxyzw": [
                0.50374569,
                -0.50228743,
                0.49632158,
                0.49760678
            ],
            "frame_id": "vlp32_1",
            "child_frame_id": "cam_fov60",
            "valid_date": "181101-181129"
        },
    ],
    "intrinsics": [
        {
            "frame_id": "cam_fov60",
            "width": 1920,
            "height": 1080,
            "D": [
                -0.5907738972958697,
                0.3105447423625289,
                -0.002406116369171268,
                -0.004505115057455168,
                0.0
            ],
            "K": [
                2031.8389140381164, 0.0, 1007.2845434678806,
                0.0, 2029.4242008945453, 503.65049480323916,
                0.0, 0.0, 1.0
            ],
            "R": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ],
            "P": [
                1718.5859375, 0.0, 1014.6422458803281, 0.0,
                0.0, 1937.922119140625, 498.91599822248827, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
        },
    ]
}
```