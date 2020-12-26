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
   <param name="cam_fx" value="906.435547"/>
   <param name="cam_fy" value="920.962402"/>
   <param name="cam_u0" value="319"/>
   <param name="cam_v0" value="180"/>
   <param name="image_width" value="640"/>
   <param name="image_height" value="480"/>
   ```
    - `sub_topic`指明订阅的图像话题。
    - `cam_fx` `cam_fy` `cam_u0` `cam_v0`为相机内参。
    - `image_width` `image_height`为相机图像尺寸。

## 运行
 - 启动`camera_pose_calibration`
   ```Shell
   roslaunch camera_pose_calibration camera_pose_calibration.launch
   ```

## 说明
 - 此程序可用来标定相机的安装俯仰角及高度。
 - 标定前需准备一个可调节高度的标定物sign，还需准备一把卷尺以及可在地上标记的粉笔。
 - 发布的图像中将显示两个圆点，红色圆点为图像主点，绿色圆点为相机等高线在图像上的投影点。
 

