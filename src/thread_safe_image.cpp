#include "thread_safe_image.h"

void ThreadSafeImage::set(const StampedImage& image)
{
    boost::unique_lock<boost::mutex> lock(_mutex);
    _image = image;
    _condition.notify_one();
}

StampedImage ThreadSafeImage::get()
{
    boost::unique_lock<boost::mutex> lock(_mutex);
    return _image;
}

StampedImage ThreadSafeImage::pop()
{
    StampedImage image;
    {
        boost::unique_lock<boost::mutex> lock(_mutex);
        while (_image.image.empty())
        {
            _condition.wait(lock);
        }
        image = _image;
        _image.image.release();
    }
    return image;
}
