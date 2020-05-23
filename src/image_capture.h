#ifndef __IMAGE_CAPTURE_H
#define __IMAGE_CAPTURE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
#include <boost/thread.hpp>
#include <mutex>
#include <atomic>

class ImageCapture 
{
public: 
    ImageCapture(const ros::NodeHandle& nh = ros::NodeHandle(), 
                 const ros::NodeHandle& private_nh = ros::NodeHandle("~")); 

    ~ImageCapture(); 

private: 
    bool Open(); 
    void Close(); 
    bool Capture(cv::Mat& image); 

    void capture_thread(); 

    bool reset_callback(std_srvs::Trigger::Request  &request, 
                        std_srvs::Trigger::Response &response);

private: 
    ros::NodeHandle _nh; 
    ros::NodeHandle _private_nh; 
    ros::Publisher _image_pub; 
    ros::ServiceServer _reset_svr; 
    
    boost::thread _thread;
    std::atomic<bool> _stop; 

    cv::VideoCapture _cap; 
    std::mutex _mutex; 

    bool _auto_reset; 
    std::shared_ptr<ros::Rate> _capture_rate; 
}; 

#endif // #ifndef __IMAGE_CAPTURE_H
