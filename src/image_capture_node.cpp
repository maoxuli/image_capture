#include <ros/ros.h>
#include "image_capture.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_capture");
    ros::NodeHandle nh, private_nh("~"); 
    ImageCapture capture(nh, private_nh); 
    ros::spin(); 
    return 0;
}
