/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:	Alberto Invernizzi                       		           *
 *   Email:     alby.inve@gmail.com                                        *
 *   Date:      08/10/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef PHOTONFOCUS_CAMERA_H
#define PHOTONFOCUS_CAMERA_H

#define BUFFER_COUNT 16 // TODO number of buffers from parameters?

#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>

#include "PvDeviceGEV.h"
#include "PvStreamGEV.h"
#include "PvPipeline.h"

// THIS MACRO EXPLOITS RESULT DESCRIPTION FOR THROWING EXCEPTIONS ON EXPR WHICH RETURNS A PvResult
#define CHECK_RESULT(expression)\
do{\
    PvResult result = expression;\
    if(!result.IsOK())\
        throw std::runtime_error(result.GetDescription().GetAscii());\
}while(false)

namespace IRALab
{

// this was imported from driver_common, since the latter seems deprecated.
enum {
  RECONFIGURE_CLOSE = 3,
  RECONFIGURE_STOP = 1,
  RECONFIGURE_RUNNING = 0,
};
class PhotonFocusCamera
{
    // FIXME it is necessary to specialize the exceptions...
    PvDevice * device;
    PvStream * stream;
    PvPipeline * pipeline;
    PvString camera_id; // IP

    PvGenParameterArray * device_parameters;
    PvGenParameterArray * stream_parameters;

    boost::shared_ptr<boost::thread> image_thread;

public:
    boost::function<void(const cv::Mat &image)> callback;

    PhotonFocusCamera(std::string ip_address);
    ~PhotonFocusCamera();

    void start();
    void stop();

    // TODO maybe the template can be re-engineered...
    template <typename ParamType,typename ValueType>
    ValueType getDeviceAttribute(std::string name, ValueType * min = NULL, ValueType * max = NULL)
    {
        if(device_parameters == NULL)
            throw std::runtime_error("Device parameters are not yet initialized.");

        ParamType * parameter = dynamic_cast<ParamType *>(device_parameters->Get(PvString(name.c_str())));

        if(parameter == NULL)
            throw std::runtime_error("Attribute " + name + " does not exist.");

        ValueType value;
        CHECK_RESULT(parameter->GetValue(value));
        if(min != NULL)
            CHECK_RESULT(parameter->GetMin(*min));
        if(max != NULL)
            CHECK_RESULT(parameter->GetMax(*max));
        return value;
    }

    template <typename ParamType,typename ValueType>
    void setDeviceAttribute(std::string name, ValueType value)
    {
        if(device_parameters == NULL)
            throw std::runtime_error("Device parameters are not yet initialized.");

        ParamType * parameter = dynamic_cast<ParamType *>(device->GetParameters()->Get(PvString(name.c_str())));

        if(parameter == NULL)
            throw std::runtime_error("Attribute " + name + " does not exist.");

        // check if the value is in the range...
        ValueType min,max;
        CHECK_RESULT(parameter->GetMin(min));
        CHECK_RESULT(parameter->GetMax(max));

        if(!(min <= value && value <= max))
        {
            std::cout << name << " is not in the range [" << min << "," << max << "]..." << std::endl;
            return;
        }

        if(!parameter->IsWritable())
        {
            std::cout << name << " is not writable at the time..." << std::endl;
            return;
        }

        CHECK_RESULT(parameter->SetValue(value));
    }
private:
    void open();
    void close();
    void acquireImages();

    PvAccessType getAccessType();
};

template <>
void PhotonFocusCamera::setDeviceAttribute<PvGenBoolean,bool>(std::string name, bool value)
{
    if(device_parameters == NULL)
        throw std::runtime_error("Device parameters are not yet initialized.");

    PvGenBoolean * parameter = dynamic_cast<PvGenBoolean *>(device->GetParameters()->Get(PvString(name.c_str())));

    if(parameter == NULL)
        throw std::runtime_error("Attribute " + name + " does not exist.");

    if(!parameter->IsWritable())
    {
        std::cout << name << " is not writable at the time..." << std::endl;
        return;
    }

    CHECK_RESULT(parameter->SetValue(value));

}

template <>
void PhotonFocusCamera::setDeviceAttribute<PvGenEnum,long>(std::string name, long value)
{
    if(device_parameters == NULL)
        throw std::runtime_error("Device parameters are not yet initialized.");

    PvGenEnum * parameter = dynamic_cast<PvGenEnum *>(device->GetParameters()->Get(PvString(name.c_str())));

    if(parameter == NULL)
        throw std::runtime_error("Attribute " + name + " does not exist.");

    // check if the value is in the range...
    long entries;
    bool is_in = false;
    CHECK_RESULT(parameter->GetEntriesCount(entries));
    for(int i=0;i<entries && !is_in;i++)
    {
        const PvGenEnumEntry * entry;
        CHECK_RESULT(parameter->GetEntryByIndex(i,&entry));
        long current_value;
        CHECK_RESULT(entry->GetValue(current_value));
        if(value == current_value)
            is_in = true;
    }

    if(!is_in)
    {
        std::cout << name << ": " << value << " is not a valid value for this enum..." << std::endl;
        return;
    }

    if(!parameter->IsWritable())
    {
        std::cout << name << " is not writable at the time..." << std::endl;
        return;
    }

    CHECK_RESULT(parameter->SetValue(value));
}

template <>
void PhotonFocusCamera::setDeviceAttribute<PvGenEnum,std::string>(std::string name, std::string value)
{
    if(device_parameters == NULL)
        throw std::runtime_error("Device parameters are not yet initialized.");

    PvGenEnum * parameter = dynamic_cast<PvGenEnum *>(device->GetParameters()->Get(PvString(name.c_str())));

    if(parameter == NULL)
        throw std::runtime_error("Attribute " + name + " does not exist.");

    // check if the value is in the range...
    long entries;
    bool is_in = false;
    CHECK_RESULT(parameter->GetEntriesCount(entries));
    for(int i=0;i<entries && !is_in;i++)
    {
        const PvGenEnumEntry * entry;
        CHECK_RESULT(parameter->GetEntryByIndex(i,&entry));
        PvString current_value;
        CHECK_RESULT(entry->GetName(current_value));
        if(value == std::string(current_value.GetAscii()))
            is_in = true;
    }

    if(!is_in)
    {
        std::cout << name << ": " << value << " is not a valid value for this enum..." << std::endl;
        return;
    }

    if(!parameter->IsWritable())
    {
        std::cout << name << " is not writable at the time..." << std::endl;
        return;
    }

    CHECK_RESULT(parameter->SetValue(PvString(value.c_str())));
}

}
#endif // PHOTONFOCUS_CAMERA_H
