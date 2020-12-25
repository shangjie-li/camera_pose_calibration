# camera_pose_calibration

ROS package for calibrating the pose of camera

## 安装
 - 建立工作空间并拷贝这个库
   ```Shell
   mkdir -p ros_ws/src
   cd ros_ws/src
   git clone https://github.com/shangjie-li/camera_pose_calibration.git
   cd ..
   catkin_make
   ```

## 参数配置
 - 修改`camera_pose_calibration/launch/camera_pose_calibration.launch`
   ```Shell
   <param name="sub_topic" value="/usb_cam/image_raw"/>
   <param name="pub_topic" value="/image_calibrator"/>
   
   <param name="cam_fx" value="906.435547"/>
   <param name="cam_fy" value="920.962402"/>
   <param name="cam_u0" value="319"/>
   <param name="cam_v0" value="180"/>
   <param name="image_width" value="640"/>
   <param name="image_height" value="480"/>
   
   <param name="depression_angle" value="-0.9"/>
   ```
    - `sub_topic`指明订阅的图像话题。
    - `pub_topic`指明发布的图像话题。
    - `cam_fx``cam_fy``cam_u0``cam_v0`为相机内参。
    - `image_width``image_height`为相机图像尺寸。
    - `depression_angle`为相机俯仰角，相机光轴低于水平面时为正，相机光轴高于水平面时为负，单位为度。

## 运行
 - 启动`camera_pose_calibration`
   ```Shell
   roslaunch camera_pose_calibration camera_pose_calibration.launch
   ```

## 说明
 - 1.发布的图像中将显示两个圆点，红色圆点为图像主点，绿色圆点为相机等高线在图像上的投影点。
 - 2.如果将相机俯仰角设置为零，则发布的图像中将只显示绿色圆点。

