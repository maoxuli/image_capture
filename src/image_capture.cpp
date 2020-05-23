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
        ROS_ERROR_STREAM("ROS exception: " << ex.what());
        throw std::runtime_error(std::string("ROS exception: ") + ex.what()); 
    }

    try 
    {
        ROS_INFO_STREAM("Start publish thread...");
        _publish_thread = boost::thread(boost::bind(&ImageCapture::publish_thread, this));

         ROS_INFO_STREAM("Start capture thread...");
        _capture_thread = boost::thread(boost::bind(&ImageCapture::capture_thread, this));
    } 
    catch (std::exception& ex) 
    {
        ROS_ERROR_STREAM("Boost thread exception: " << ex.what());
        throw std::runtime_error(std::string("Boost thread exception: ") + ex.what());
    }
}

ImageCapture::~ImageCapture() 
{
    Close(); 

    if (_capture_thread.joinable())
    {
        _capture_thread.interrupt();
        _capture_thread.join();
    }

    if (_publish_thread.joinable())
    {
        _publish_thread.interrupt();
        _publish_thread.join();
    }
}

bool ImageCapture::reset_callback(std_srvs::Trigger::Request &request, 
                                  std_srvs::Trigger::Response &response)
{
    try
    {
        ROS_INFO("Reset service callback...");
        Close();
        Open(); 
        response.success = true; 
        return true; 
    }
    catch (const cv::Exception& ex)
    {
        ROS_ERROR_STREAM("OpenCV exception: " << ex.what());
        response.success = false; 
        response.message = std::string("OpenCV exception: ") + ex.what(); 
        return false;
    }
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

    // backend of OpenCV
    std::string backend; 
    LoadParam(_private_nh, "backend", backend);
    
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
        if (backend == "gstreamer") {
            _cap.open(provider, cv::CAP_GSTREAMER);
        }
        else {
            _cap.open(provider);
        }
 
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

void ImageCapture::capture_thread() 
{  
    ROS_INFO("Capture thread start...");
    Open(); 

    cv::Mat image;
    double start_time = ros::Time::now().toNSec(); 
    int frame_count = 0; 

    try 
    {
        while (ros::ok()) 
        {
            try 
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

                _queued_image.set(StampedImage(image, ros::Time::now().toSec()));
            }
            catch (const cv::Exception& ex)
            {
                ROS_ERROR_STREAM("OpenCV exception: " << ex.what());
                continue; 
            }

            frame_count++; 
            double stop_time = ros::Time::now().toNSec(); 
            if (stop_time - start_time > 1000000000)
            {
                start_time = stop_time; 
                ROS_INFO_STREAM("Capture FPS: " << frame_count); 
                frame_count = 0; 
            }
        }
    }
    catch (const boost::thread_interrupted&)
    {
    }

    Close(); 
    ROS_INFO("Capture thread stopped!");
}

void ImageCapture::publish_thread() 
{  
    ROS_INFO("Publsih thread start...");
    cv_bridge::CvImage cvi; 
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    double start_time = ros::Time::now().toNSec(); 
    int frame_count = 0; 

    try 
    {
        while (ros::ok()) 
        {
            try 
            {
                StampedImage si = _queued_image.pop();
                cvi.image = si.image; 
                cvi.header.stamp = ros::Time(si.stamp);

                assert(_image_pub); 
                _image_pub.publish(cvi.toImageMsg()); 
                ROS_DEBUG("Publish image: %f", cvi.header.stamp.toSec());
            }
            catch(const cv_bridge::Exception& ex) {
                ROS_WARN_STREAM("cv_bridge exception: " << ex.what());
            }
            catch (const ros::Exception& ex) {
                ROS_ERROR_STREAM("ROS exception: " << ex.what());
            }

            frame_count++; 
            double stop_time = ros::Time::now().toNSec(); 
            if (stop_time - start_time > 1000000000)
            {
                start_time = stop_time; 
                ROS_INFO_STREAM("Publish FPS: " << frame_count); 
                frame_count = 0; 
            }
        }
    }
    catch (const boost::thread_interrupted&)
    {
    }
    ROS_INFO("Publish thread stopped!");
}
