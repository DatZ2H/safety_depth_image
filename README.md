# safety_depth_image

# Require
- Hardware:
    + Ubuntu 18.04 - ROS Melodic
    + Depth camera: Intel Realsense Depth Camera D435i
- Install python:
    + $sudo apt install python-pip
- Install Intel Realsense SDK:
    + Install: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
    + Test camera: realsense-viewer
- Install driver Realsense camera with ROS:
    + $sudo apt install ros-melodic-realsense2-camera
- Install driver numpy with ROS:
    + $sudo apt install ros-melodic-ros-numpy

- Permit access: 
    + Direct to folder: /safety_depth_image/detect_obstacle/script
    + $sudo chmod +x detect_obstacle_depth_camera.py multiple_depth_camera.py

# File structure
```t
    detect_obstacle
    ├── launch
        └── detect_obstacle.launch
        └── detect_with_two_camera.launch
    ├── msg
        └── fields_safety.msg
    ├── rviz
        └── depth_image.rviz
    ├── scripts
        └── detect_obstacle_depth_camera.py
        └── multiple_depth_camera.py
    ├── CMakeLists.txt
    └── package.xml
```

# Nodes 
1. detect_obstacle
- Subscribed Topics:
    + [camera_name]/color/camera_info (sensor_msgs/CameraInfo): Thông tin của depth camera
    + [camera_name]/color/image_raw (sensor_msgs/Image): Dữ liệu hình ảnh của camera (color camera)
    + [camera_name]/depth/image_rect_raw (sensor_msgs/Image): Dữ liệu khoảng cách của camera (depth camera)
- Published Topics:
    + [camera_name]/color_image_message (sensor_msgs/Image): Dữ liệu hình ảnh của camera (color camera) sau khi biến đổi
    + [camera_name]/depth_image_message (sensor_msgs/Image): Dữ liệu khoảng cách của camera (depth camera) sau khi biến đổi
    + [camera_name]/fields_safety (detect_obstacle/fields_safety): Thông tin vùng của safety

2. multiple_depth_camera
- Subscribed Topics:
    + [camera_name]/fields_safety (detect_obstacle/fields_safety): Thông tin vùng của safety của camera thứ 1
    + [camera_name]/fields_safety (detect_obstacle/fields_safety): Thông tin vùng của safety của camera thứ 2
- Published Topics:
    + [camera_name]/fields_safety (detect_obstacle/fields_safety): Thông tin vùng của safety sau khi kết hợp hai camera

3. Parameters
- use_detect (bool, default: "true"): Cho phép sử dụng depth camera để  xác định vật cản hay không?
- use_rotate_90_counter_clockwise (bool, default: "true"): Cho phép xoay ảnh ngược chiều kim đồng hồ một góc 90 độ hay không?
- is_display_origin_color_image (bool, default: "false"): Cho phép hiển thị hình ảnh màu hay không?
- is_display_origin_depth_image (bool, default: "false"): Cho phép hiển thị hình ảnh depth image gốc hay không?
- is_display_resize_depth_image (bool, default: "false"): Cho phép hiển thị hình ảnh depth image sau khi resize hay không?
- camera (string, default: "/depth_camera"): Tên của camera. Đây chính là camera_name của Subscribed Topics và Published Topics. Sử dụng khi dùng 1 camera hoặc kết hợp nhiều camera
- topic_camera_info_sub (string, default: "/color/camera_info"): Tên topic chứa thông tin chung về camera
- topic_color_image_sub (string, default: "/color/image_raw"): Tên topic chứa dữ liệu về  hình ảnh camera
- topic_depth_image_sub (string, default: "/depth/image_rect_raw"): Tên topic chứa dữ liệu về  depth camera
- top_image_resize (int, default: "238"): Số hàng pixcel của depth camera (tính từ tâm image lên cạnh trên) sau khi resize
- bottom_image_resize (int, default: "238"): Số hàng pixcel của depth camera (tính từ tâm image lên cạnh dưới) sau khi resize
- left_image_resize (int, default: "200"): Số cột pixcel của depth camera (tính từ tâm image lên cạnh trái) sau khi resize
- right_image_resize (int, default: "200"): Số cột pixcel của depth camera (tính từ tâm image lên cạnh phải) sau khi resize
- distance_field_detect (int, default: "1.7"): Khoảng cách (m) của vùng Detect
- distance_field_warning (int, default: "1.2"): Khoảng cách (m) của vùng Warning
- distance_field_dangerous (int, default: "0.7"): Khoảng cách (m) của vùng Dangerous

- camera_left (string, default: "/depth_camera_left"): Tên của camera bên trái. Đây chính là camera_name của Subscribed Topics và Published Topics. Sử dụng khi dùng nhiều camera
- camera_right (string, default: "/depth_camera_right"): Tên của camera bên phải. Đây chính là camera_name của Subscribed Topics và Published Topics. Sử dụng khi dùng nhiều camera

# Message
```t
    fields_safety
    ├── header
        └── uint32 seq
        └── time stamp
            └── uint32 secs
            └── uint32 nsecs
        └── string frame_id
    ├── bool enable
    └── bool[] fields               # [detect, warning, dangerous]
```

# Run example
1. Setup
- Kiểm tra kết nốí của camera: 
    + $ls /dev/video*
    + Expect: /dev/video0  /dev/video1  /dev/video2
- Config camera: $realsense-viewer

2. Run example
- Chạy chương trình với 1 camera:
    + $roslaunch detect_obstacle detect_obstacle.launch
- Chạy chương trình với 2 camera:
    + $roslaunch detect_obstacle detect_with_two_camera.launch

3. Check
- Lắng nghe topic: /[camera_name]/fields_safety
    + $rostopic echo /depth_camera/fields_safety
    + Expect: 
```t
    header:
        seq: 30
        stamp:
            secs: 0
            nsecs: 0
        frame_id: "/depth_camera"
    enable: True
    fields: [False, False, True]
```
