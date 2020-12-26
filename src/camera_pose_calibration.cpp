#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>

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
	
	double set_parameter(double h1, double h2, double d);
	void set_parameter(double angle);
	
	bool compute_depression_angle_;
	bool show_center_point_;
    bool show_height_point_;
	
private:
	void image_cb(const sensor_msgs::Image::ConstPtr &msg);
	
	std::string sub_topic_;
	double cam_fx_;
    double cam_fy_;
    int cam_u0_;
    int cam_v0_;
    int image_width_;
    int image_height_;
	
	double depression_angle_;
	
	ros::Subscriber sub_image_;
};

Calibrator::Calibrator(ros::NodeHandle &nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/usb_cam/image_raw");
    nh.param<double>("cam_fx", cam_fx_, 581.0);
    nh.param<double>("cam_fy", cam_fy_, 604.0);
    nh.param<int>("cam_u0", cam_u0_, 605.0);
    nh.param<int>("cam_v0", cam_v0_, 332.0);
    nh.param<int>("image_width", image_width_, 1280);
    nh.param<int>("image_height", image_height_, 720);
    
	sub_image_ = nh.subscribe(sub_topic_, 1, &Calibrator::image_cb, this);
}

Calibrator::~Calibrator()
{
}

double Calibrator::set_parameter(double h1, double h2, double d)
{
    depression_angle_ = atan((h1 - h2) / d);
    return depression_angle_;
}

void Calibrator::set_parameter(double angle)
{
    depression_angle_ = angle;
}

void Calibrator::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat img = cv_ptr->image;
	
	int r0 = 255;
    int g0 = 0;
    int b0 = 0;
    
    int r = 0;
    int g = 255;
    int b = 0;
    
    int radius = 3;
    int thickness = 1;
    int lineType = 8;
    
    if (show_center_point_ || show_center_point_)
    {
        cv::namedWindow("calibrator", cv::WINDOW_NORMAL);
    }
    
    if (show_center_point_)
    {
        if (cam_u0_ < 0 || cam_u0_ >= image_width_ || cam_v0_ < 0 || cam_v0_ >= image_height_)
        {
            ROS_ERROR("Cannot draw cv::Point (%d, %d) in the image!", cam_u0_, cam_v0_);
        }
        cv::circle(img, cv::Point(cam_u0_, cam_v0_), radius, cv::Scalar(b0, g0, r0), thickness, lineType);
    }
    
    if (show_height_point_)
    {
        double theta;
        int uu, vv;
        
        theta = depression_angle_ * PI / 180;
        uu = cam_u0_;
        vv = cam_v0_ + cam_fy_ * tan(theta);
    
        if (uu < 0 || uu >= image_width_ || vv < 0 || vv >= image_height_)
        {
            ROS_ERROR("Cannot draw cv::Point (%d, %d) in the image!", uu, vv);
        }
        cv::circle(img, cv::Point(uu, vv), radius, cv::Scalar(b, g, r), thickness, lineType);
    }
    
    cv::imshow("calibrator", img);
    if (cv::waitKey(1) == 27)
    {
        cv::destroyWindow("calibrator");
        ros::shutdown();
    };
}

void chatter(Calibrator* p)
{
    std::string str;
    bool str_state;
    
    std::cout << "" << std::endl;
    std::cout << "---Calibrator activated!---" << std::endl;
    
    std::cout << "" << std::endl;
    std::cout << "Do you know the depression angle of the camera?" << std::endl;
    std::cout << "Enter Y, if you know and then calibrate the height of the camera." << std::endl;
    std::cout << "Enter N, if you don't know and then calibrate the depression angle first." << std::endl;
    
    str_state = false;
    while (!str_state)
    {
        std::cin >> str;
        if (str == "Y" || str == "y")
        {
            p->compute_depression_angle_ = false;
            str_state = true;
        }
        else if (str == "N" || str == "n")
        {
            p->compute_depression_angle_ = true;
            str_state = true;
        }
        else
        {
            std::cout << "Please enter Y or N to move on." << std::endl;
        }
    }
    
    if (p->compute_depression_angle_)
    {
        std::cout << "" << std::endl;
        std::cout << "Put the sign on the position 1 (closer to camera)." << std::endl;
        std::cout << "Make sure the sign coincides exactly with the red circle in the image." << std::endl;
        std::cout << "Mark the position." << std::endl;
        
        std::cout << "" << std::endl;
        std::cout << "Enter Y when you finish this job." << std::endl;
    
        str_state = false;
        while (!str_state)
        {
            std::cin >> str;
            if (str == "Y" || str == "y")
            {
                str_state = true;
            }
            else
            {
                std::cout << "Enter Y when you finish this job." << std::endl;
            }
        }
        
        std::cout << "" << std::endl;
        std::cout << "What's the height of the sign?" << std::endl;
        double h1;
        std::cin >> h1;
        
        std::cout << "" << std::endl;
        std::cout << "Put the sign on the position 2 (further from camera)." << std::endl;
        std::cout << "Make sure the sign coincides exactly with the red circle in the image." << std::endl;
        std::cout << "Mark the position." << std::endl;
        
        std::cout << "" << std::endl;
        std::cout << "Enter Y when you finish this job." << std::endl;
    
        str_state = false;
        while (!str_state)
        {
            std::cin >> str;
            if (str == "Y" || str == "y")
            {
                str_state = true;
            }
            else
            {
                std::cout << "Enter Y when you finish this job." << std::endl;
            }
        }
        
        std::cout << "" << std::endl;
        std::cout << "What's the height of the sign?" << std::endl;
        double h2;
        std::cin >> h2;
        
        std::cout << "" << std::endl;
        std::cout << "What's the distance between the position 1 and the position 2?" << std::endl;
        double d;
        std::cin >> d;
        
        double angle;
        angle = p->set_parameter(h1, h2, d);
        
        std::cout << "" << std::endl;
        std::cout << "The depression angle of the camera is " << angle << "." << std::endl;
        
        p->show_height_point_ = true;
        p->show_center_point_ = false;
        
        std::cout << "" << std::endl;
        std::cout << "Put the sign to coincide exactly with the green circle in the image." << std::endl;
        std::cout << "Measure the height of the sign, which is equal to the height of the camera." << std::endl;
        std::cout << "And then you can press Esc on the image to shut down this procedure." << std::endl;
    }
    else
    {
        std::cout << "" << std::endl;
        std::cout << "What's the depression angle of the camera?" << std::endl;
        double angle;
        std::cin >> angle;
        p->set_parameter(angle);
        
        p->show_height_point_ = true;
        p->show_center_point_ = false;
        
        std::cout << "" << std::endl;
        std::cout << "Put the sign to coincide exactly with the green circle in the image." << std::endl;
        std::cout << "Measure the height of the sign, which is equal to the height of the camera." << std::endl;
        std::cout << "And then you can press Esc on the image to shut down this procedure." << std::endl;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibrator");
    ros::NodeHandle nh("~");
    
    Calibrator* cp = new Calibrator(nh);
    cp->show_center_point_ = true;
    cp->show_height_point_ = false;
    
    boost::thread server(chatter, cp);
    
    ros::spin();
    
    return 0;
}

