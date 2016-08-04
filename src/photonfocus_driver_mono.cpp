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

#include <ros/ros.h>
#include <signal.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <photonfocus_camera.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <ira_photonfocus_driver/photonfocus_monoConfig.h>

#include <camera_info_manager/camera_info_manager.h>

#include <boost/scoped_ptr.hpp>

namespace IRALab
{
class PhotonFocusDriver
{
private:
    // ROS
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport;
    image_transport::CameraPublisher publisher;

    // ROS Message
    sensor_msgs::ImagePtr image;

    // PhotonFocus Camera
    boost::scoped_ptr<IRALab::PhotonFocusCamera> camera;
    std::string camera_name;

    // TODO Dynamic Reconfigure [with parameter server]
    dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig> reconfig_svr_;

    // Calibration Manager
    boost::shared_ptr<camera_info_manager::CameraInfoManager> calibration_manager;

public:
    PhotonFocusDriver(std::string camera_name,std::string ip, const ros::NodeHandle & node_handle):
        node_handle(node_handle),
        image_transport(node_handle),
        camera_name(camera_name),
        calibration_manager(new camera_info_manager::CameraInfoManager(node_handle,camera_name))
    {
        publisher = image_transport.advertiseCamera("image_raw",1);

        camera.reset(new IRALab::PhotonFocusCamera(ip));
        camera->start();
        camera->callback = boost::bind(&PhotonFocusDriver::publishImage, this, _1);
        // TODO parameter server callback
        reconfig_svr_.setCallback(boost::bind(&PhotonFocusDriver::configCb, this, _1, _2));

        std::cout << std::setw(80) << std::setfill(' ') << std::left << "===== PhotonFocus Camera ----- START ===== " << std::endl;
    }

    ~PhotonFocusDriver()
    {
        camera->stop();
        camera.reset();
        std::cout << "===== PhotonFocus Camera ----- STOP  ===== " << std::endl;
    }

    void publishImage(const cv::Mat img)
    {
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = img;
        cv_image.header.stamp = ros::Time::now();
        image = cv_image.toImageMsg();

        sensor_msgs::CameraInfo::Ptr camera_info;
        if(calibration_manager->isCalibrated()) // calibration exists
            camera_info.reset(new sensor_msgs::CameraInfo(calibration_manager->getCameraInfo()));
        else // calibration doesn't exist
        {
            camera_info.reset(new sensor_msgs::CameraInfo());
            camera_info->width = image->width;
            camera_info->height = image->height;
        }

		// WARNING for calibration with cameracalibrator for ROS replace "stereo_rig" with ("/" + camera_name) in both next 2 lines
        image->header.frame_id = "stereo_rig"; 
        camera_info->header.frame_id = "stereo_rig";
        camera_info->header.stamp = cv_image.header.stamp;

        publisher.publish(image,camera_info);
    }

    void configCb(photonfocus_camera::photonfocus_monoConfig & config, uint32_t level)
    {
        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera->stop();

        camera->setDeviceAttribute<PvGenEnum,std::string>("PixelFormat","Mono8");

        //# ----- Image Size Control -----
        camera->setDeviceAttribute<PvGenInteger,long>("Width",config.Width*32+768);
        camera->setDeviceAttribute<PvGenInteger,long>("Height",config.Height);

        camera->setDeviceAttribute<PvGenInteger,long>("OffsetX",config.OffsetX*32);
        camera->setDeviceAttribute<PvGenInteger,long>("OffsetY",config.OffsetY);

        //# ----- Exposure and FrameRate -----
        camera->setDeviceAttribute<PvGenFloat,double>("ExposureTimeAbs",config.ExposureTimeAbs);
        camera->setDeviceAttribute<PvGenBoolean,bool>("ConstantFramerate_CFR",config.ConstantFramerate_CFR);
        if(config.ConstantFramerate_CFR)
            camera->setDeviceAttribute<PvGenFloat,double>("Frametime",config.Frametime);

        camera->setDeviceAttribute<PvGenBoolean,bool>("Trigger_Interleave",config.Trigger_Interleave);
        if(!config.Trigger_Interleave)\
        {
            camera->setDeviceAttribute<PvGenEnum,long>("LinLog_Mode",config.LinLog_Mode);
            if(config.LinLog_Mode == 4)
            {
                std::cout << "UserDefined" << std::endl;
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value1",config.LinLog_Value1);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Value2",config.LinLog_Value2);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time1",config.LinLog_Time1);
                camera->setDeviceAttribute<PvGenInteger,long>("LinLog_Time2",config.LinLog_Time2);
            }
        }
        camera->setDeviceAttribute<PvGenInteger,long>("Voltages_BlackLevelOffset",config.Voltages_BlackLevelOffset);

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera->start();
    }

};
}

bool ros_shutdown = false;

void signal_handler(int sig_code)
{
    ros_shutdown = true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ira_photonfocus");
    ros::NodeHandle node_handle("~");

    signal(SIGINT,signal_handler);

    std::string ip;

    if(!node_handle.getParam("ip",ip))
    {
        std::cout << "Usage: " << argv[0] << " _ip:=IP_ADDRESS" << std::endl;
        return 0;
    }

    std::string camera_name = ros::this_node::getName();
    camera_name = std::string(camera_name.begin()+ros::this_node::getNamespace().length(),camera_name.end());

    boost::shared_ptr<IRALab::PhotonFocusDriver> camera_node(new IRALab::PhotonFocusDriver(camera_name,ip,node_handle));

    while(ros::ok() && !ros_shutdown)
        ros::spinOnce();

    // the node is shutting down...cleaning
    camera_node.reset();

    ros::shutdown();
    return 0;
}
