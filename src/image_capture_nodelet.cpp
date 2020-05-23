#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "image_capture.h"

namespace image_capture
{

class ImageCaptureNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        _capture.reset(new ImageCapture(getNodeHandle(), getPrivateNodeHandle()));  
    }

private:
    boost::shared_ptr<ImageCapture> _capture;
};

} // namespace image_capture

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_capture::ImageCaptureNodelet, nodelet::Nodelet)
