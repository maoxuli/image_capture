#ifndef __THREAD_SAFE_IMAGE_H
#define __THREAD_SAFE_IMAGE_H 

#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

struct StampedImage 
{
    cv::Mat image; 
    double stamp; 

    StampedImage() 
    {

    }

    StampedImage(const cv::Mat& _image, double _stamp)
    : image(_image)
    , stamp(_stamp)
    {
        
    }
}; 

class ThreadSafeImage
{
public:
    void set(const StampedImage& image);
    StampedImage get();
    StampedImage pop();

private:
    boost::mutex _mutex;
    boost::condition_variable _condition;
    StampedImage _image;
};

#endif // #ifndef __THREAD_SAFE_IMAGE_H
