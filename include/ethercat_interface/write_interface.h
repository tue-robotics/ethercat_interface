#ifndef WRITE_INTERFACE_H
#define WRITE_INTERFACE_H

#include "ethercat_interface/ethercat_includes.h"
#include <memory>

//template <typename T>
//class WriteImpl
//{
//public:
//    virtual bool write(double value) = 0;
//};  // End of class WriteImpl


class WriteInterface
{
public:

    virtual bool write(double value) = 0;
//    bool init(int slave, int channel)
//    {
//        // Get endpoint from slave and channel

//        //if (16 bit)
////        impl_ = std::unique_ptr(new WriteImpl<int16>());
//    }

//    bool write(double value) {
//        return impl_->write(value);
//    }
//private:
//    std::unique_ptr<WriteImpl> impl_;
};  // End of class WriteInterface

#endif // WRITE_INTERFACE_H
