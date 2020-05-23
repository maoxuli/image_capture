#include "image_capture.h"
#include "ros_parameter.hpp"

#include <cv_bridge/cv_bridge.h>

ImageCapture::ImageCapture(const ros::NodeHandle& nh, 
                           const ros::NodeHandle& private_nh) 
: _nh(nh)
, _private_nh(private_nh)
{
    try 
    {
        std::string reset_service = "reset";
        ROS_INFO_STREAM("Advertise reset service: " << reset_service);
        _reset_svr = _nh.advertiseService(reset_service, &ImageCapture::reset_callback, this);
        
        std::string image_topic = "image"; 
        ROS_INFO_STREAM("Advertise image topic: " << image_topic);
        _image_pub = _nh.advertise<sensor_msgs::Image>(image_topic, 2);
    }
    catch (const ros::Exception& ex)
    {
        ROS_ERROR("ROS exception: %s", ex.what());
        throw std::runtime_error(std::string("ROS exception: ") + ex.what()); 
    }

    try 
    {
        _stop = false; 
        ROS_INFO_STREAM("Start capture thread...");
        _thread = boost::thread(boost::bind(&ImageCapture::capture_thread, this));
    } 
    catch (std::exception& ex) 
    {
        ROS_ERROR("Failed to start thread: %s", ex.what());
        throw std::runtime_error(std::string("Boost thread exception: ") + ex.what());
    }
}

ImageCapture::~ImageCapture() 
{
    Close(); 
    _stop = true; 
    _thread.join(); 
}

bool ImageCapture::reset_callback(std_srvs::Trigger::Request &request, 
                                  std_srvs::Trigger::Response &response)
{
    ROS_INFO("Reset service callback...");
    Close();
    Open(); 
    response.success = true; 
    return true; 
}

// Open stream for capture 
// Refresh parameters for every open  
bool ImageCapture::Open() 
{
    ROS_INFO("Opening stream...");
    std::lock_guard<std::mutex> lock(_mutex); 

    // auto reset on failure or end of stream 
    _auto_reset = true; 
    LoadParam(_private_nh, "auto_reset", _auto_reset); 

    // image provider 
    std::string provider;
    LoadParam(_private_nh, "provider", provider, true); 
    
    // image size  
    int width = 0, height = 0; 
    LoadParam(_private_nh, "width", width); 
    LoadParam(_private_nh, "height", height); 

    // frame rate  
    double fps = 0; 
    LoadParam(_private_nh, "fps", fps); 

    try 
    {
        int device_id = std::stoi(provider);
        ROS_INFO_STREAM("Stream provider: /dev/video" << device_id);
        _cap.open(device_id);
    } 
    catch (std::invalid_argument& ex) 
    {
        ROS_INFO_STREAM("Stream provider: " << provider);
        _cap.open(provider);
 
    }

    if (!_cap.isOpened()) 
    {
        ROS_ERROR_STREAM("Failed to open stream provider: " << provider);
        return false;
    }

    if (width > 0) 
    {
        if (!_cap.set(cv::CAP_PROP_FRAME_WIDTH, width)) {
            ROS_WARN("Width is not support!");
        }
        else {
            ROS_INFO_STREAM("Width is set to: " << _cap.get(cv::CAP_PROP_FRAME_WIDTH));
        }
    }

    if (height > 0) 
    {
        if (!_cap.set(cv::CAP_PROP_FRAME_HEIGHT, height)) {
            ROS_WARN("Height is not support!");
        }
        else {
            ROS_INFO_STREAM("Height is set to: " << _cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        }
    }

    if (fps > 0) 
    {
        if (!_cap.set(cv::CAP_PROP_FPS, fps)) {
            ROS_WARN("FPS is not support!");
        }
        else {
            ROS_INFO_STREAM("FPS is set to: " << _cap.get(cv::CAP_PROP_FPS));
        }
    }

    _capture_rate.reset(); 
    if (fps > 0) _capture_rate.reset(new ros::Rate(fps)); 

    ROS_INFO("Stream opened.");
    return true;
}

void ImageCapture::Close() 
{
    ROS_INFO("Closing stream...");
    std::lock_guard<std::mutex> lock(_mutex); 
    _cap.release(); 
    ROS_INFO("Stream closed!");
}

bool ImageCapture::Capture(cv::Mat& image) 
{
    ROS_DEBUG("Capture image...");
    std::lock_guard<std::mutex> lock(_mutex); 
    return _cap.isOpened() && _cap.read(image); 
}

// thread function for capture and publish 
void ImageCapture::capture_thread() 
{  
    ROS_INFO("Capture thread start...");
    Open(); 

    // loop to capture and publish image 
    cv::Mat image;
    cv_bridge::CvImage cvi; 
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    while (!_stop && ros::ok()) 
    {
        if (!Capture(image)) 
        {
            ROS_WARN("Failed to capture image!");
            ros::Duration(0.5).sleep();
            if (_auto_reset) 
            {
                Close(); 
                ros::Duration(0.5).sleep();
                Open(); 
            }
            continue; 
        }

        cvi.header.stamp = ros::Time::now(); 
        cvi.image = image; 

        assert(_image_pub); 
        _image_pub.publish(cvi.toImageMsg()); 
        ROS_DEBUG("Publish image: %f", cvi.header.stamp.toSec());

        if (_capture_rate) _capture_rate->sleep(); 
    }

    Close(); 
    ROS_INFO("Capture thread stopped!");
}
