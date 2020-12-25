#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <std_msgs/Int32.h>
#include <string>
#include <vector>

#include <cmath>

#define PI 3.1415926

class Calibrator
{
public:
	Calibrator(ros::NodeHandle &nh);
	~Calibrator();
	
private:
	void image_cb(const sensor_msgs::Image::ConstPtr &msg);
	void draw(cv::Mat &img);
	
private:
	std::string sub_topic_;
	std::string pub_topic_;
	
	double cam_fx_;
    double cam_fy_;
    int cam_u0_;
    int cam_v0_;
        
    int image_width_;
    int image_height_;
    
    double depression_angle_;
	
	ros::Subscriber sub_image_;
	ros::Publisher pub_image_;
};

Calibrator::Calibrator(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/usb_cam/image_raw");
    nh.param<std::string>("pub_topic", pub_topic_, "/image_calibrator");
    
    nh.param<double>("cam_fx", cam_fx_, 581.0);
    nh.param<double>("cam_fy", cam_fy_, 604.0);
    nh.param<int>("cam_u0", cam_u0_, 605.0);
    nh.param<int>("cam_v0", cam_v0_, 332.0);
    
    nh.param<int>("image_width", image_width_, 1280);
    nh.param<int>("image_height", image_height_, 720);
    
    nh.param<double>("depression_angle", depression_angle_, 1.25);
	
	sub_image_ = nh.subscribe(sub_topic_, 1, &Calibrator::image_cb, this);
	pub_image_ = nh.advertise<sensor_msgs::Image>(pub_topic_, 1);
	
    ros::spin();
}

Calibrator::~Calibrator()
{
}

void Calibrator::draw(cv::Mat &img)
{
    int r0 = 255;
    int g0 = 0;
    int b0 = 0;
    
    int r = 0;
    int g = 255;
    int b = 0;
    
    int radius = 3;
    int thickness = 1;
    int lineType = 8;
    
    double theta;
    int uu, vv;
    
    theta = depression_angle_ * PI / 180;
    uu = cam_u0_;
    vv = cam_v0_ - cam_fy_ * tan(theta);
    
    if (cam_u0_ < 0 || cam_u0_ >= image_width_ || cam_v0_ < 0 || cam_v0_ >= image_height_)
    {
        ROS_ERROR("Cannot draw cv::Point (%d, %d) in the image!", cam_u0_, cam_v0_);
    }
    cv::circle(img, cv::Point(cam_u0_, cam_v0_), radius, cv::Scalar(r0, g0, b0), thickness, lineType);
    
    if (uu < 0 || uu >= image_width_ || vv < 0 || vv >= image_height_)
    {
        ROS_ERROR("Cannot draw cv::Point (%d, %d) in the image!", uu, vv);
    }
    cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(r, g, b), thickness, lineType);
}

void Calibrator::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
	cv::Mat img = cv_ptr->image;
	
	draw(img);
	
	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "rgb8", img).toImageMsg();
	pub_image_.publish(msg_out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
    ros::NodeHandle nh("~");

    Calibrator calibrator(nh);
    return 0;
}

