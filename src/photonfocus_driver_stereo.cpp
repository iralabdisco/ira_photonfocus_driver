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
#include <mutex>

namespace IRALab
{
class PhotonFocusDriver
{
private:
    // ROS
    ros::NodeHandle node_handle_l;
    ros::NodeHandle node_handle_r;
    image_transport::ImageTransport image_transport_l;
    image_transport::ImageTransport image_transport_r;
    image_transport::CameraPublisher publisher_l;
    image_transport::CameraPublisher publisher_r;

    // ROS Message
    sensor_msgs::ImagePtr image_l;
    sensor_msgs::ImagePtr image_r;


    // PhotonFocus Camera
    boost::scoped_ptr<IRALab::PhotonFocusCamera> camera_l;
    std::string camera_name_l;
    boost::scoped_ptr<IRALab::PhotonFocusCamera> camera_r;
    std::string camera_name_r;

    // TODO Dynamic Reconfigure [with parameter server]
    boost::scoped_ptr<dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig> > reconfig_svr_l;
    boost::scoped_ptr<dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig> > reconfig_svr_r;

    // Calibration Manager
    boost::shared_ptr<camera_info_manager::CameraInfoManager> calibration_manager_l;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> calibration_manager_r;

    //Sync
    std::vector<sensor_msgs::Image> l_imgs_buffer, r_imgs_buffer;
    std::mutex mutex_l, mutex_r;

public:
    PhotonFocusDriver(std::string camera_name_l, std::string camera_name_r, std::string ip_l, std::string ip_r, const ros::NodeHandle & node_handle_l, const ros::NodeHandle & node_handle_r):
        node_handle_l(node_handle_l),
        image_transport_l(node_handle_l),
        node_handle_r(node_handle_r),
        image_transport_r(node_handle_r),
        camera_name_l(camera_name_l),
        camera_name_r(camera_name_r),
        calibration_manager_l(new camera_info_manager::CameraInfoManager(node_handle_l,camera_name_l)),
        calibration_manager_r(new camera_info_manager::CameraInfoManager(node_handle_r,camera_name_r))
    {
        publisher_l = image_transport_l.advertiseCamera("image_raw",1);
        publisher_r = image_transport_r.advertiseCamera("image_raw",1);

        camera_l.reset(new IRALab::PhotonFocusCamera(ip_l));
        camera_l->start();
        camera_l->callback = boost::bind(&PhotonFocusDriver::publishImage_l, this, _1);
        reconfig_svr_l.reset(new dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig>(node_handle_l));
        reconfig_svr_l -> setCallback(boost::bind(&PhotonFocusDriver::configCb_l, this, _1, _2));

        camera_r.reset(new IRALab::PhotonFocusCamera(ip_r));
        camera_r->start();
        camera_r->callback = boost::bind(&PhotonFocusDriver::publishImage_r, this, _1);
        // TODO parameter server callback
        // TODO parameter server callback
        reconfig_svr_r.reset(new dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig>(node_handle_r));
        dynamic_reconfigure::Server<photonfocus_camera::photonfocus_monoConfig>::CallbackType f_r;
        f_r = boost::bind(&PhotonFocusDriver::configCb_r, this, _1, _2);
        reconfig_svr_r -> setCallback(f_r);

        std::cout << std::setw(80) << std::setfill(' ') << std::left << "===== PhotonFocus Camera ----- START ===== " << std::endl;
    }

    ~PhotonFocusDriver()
    {
        camera_l->stop();
        camera_l.reset();
        camera_r->stop();
        camera_r.reset();
        std::cout << "===== PhotonFocus Camera ----- STOP  ===== " << std::endl;
    }

    void publishImage_l(const cv::Mat img)
    {

        //std::cout<<"LEFT\n";
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = img;
        cv_image.header.stamp = ros::Time::now();
        image_l = cv_image.toImageMsg();

        /*sensor_msgs::CameraInfo::Ptr camera_info;
        if(calibration_manager_l->isCalibrated()) // calibration exists
            camera_info.reset(new sensor_msgs::CameraInfo(calibration_manager_l->getCameraInfo()));
        else // calibration doesn't exist
        {
            camera_info.reset(new sensor_msgs::CameraInfo());
            camera_info->width = image_l->width;
            camera_info->height = image_l->height;
        }*/

		// WARNING for calibration with cameracalibrator for ROS replace "stereo_rig" with ("/" + camera_name) in both next 2 lines
        image_l->header.frame_id = "stereo_rig";
        //camera_info->header.frame_id = "stereo_rig";
        //camera_info->header.stamp = cv_image.header.stamp;

        mutex_r.lock();
        if(r_imgs_buffer.size() > 0) {
            ros::Time ros_time = ros::Time::now();
            sensor_msgs::Image local_r = r_imgs_buffer.at(r_imgs_buffer.size()-1);
            if(ros_time - local_r.header.stamp < ros::Duration(0.01)) {
                r_imgs_buffer.clear();
                mutex_r.unlock();

                sensor_msgs::CameraInfo::Ptr camera_info_l, camera_info_r;
                camera_info_l.reset(new sensor_msgs::CameraInfo(calibration_manager_l->getCameraInfo()));
                camera_info_r.reset(new sensor_msgs::CameraInfo(calibration_manager_r->getCameraInfo()));
                camera_info_l->header.frame_id = "stereo_rig";
                camera_info_r->header.frame_id = "stereo_rig";
                camera_info_l->header.stamp = ros_time;
                camera_info_r->header.stamp = ros_time;

                image_l->header.frame_id = "stereo_rig";
                image_l->header.stamp = ros_time;
                local_r.header.stamp = ros_time;

                publisher_r.publish(local_r,*camera_info_r);
                publisher_l.publish(image_l,camera_info_l);
            }
            else {
                mutex_r.unlock();
                mutex_l.lock();
                l_imgs_buffer.clear();
                l_imgs_buffer.push_back(*image_l);
                mutex_l.unlock();
            }

        }
        else {
            mutex_r.unlock();
            mutex_l.lock();
            l_imgs_buffer.clear();
            l_imgs_buffer.push_back(*image_l);
            mutex_l.unlock();
        }

        //publisher_l.publish(image_l,camera_info);
    }

    void publishImage_r(const cv::Mat img)
    {
        //std::cout<<"RIGHT\n";
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = img;
        cv_image.header.stamp = ros::Time::now();
        image_r = cv_image.toImageMsg();

        /*sensor_msgs::CameraInfo::Ptr camera_info;
        if(calibration_manager_r->isCalibrated()) // calibration exists
            camera_info.reset(new sensor_msgs::CameraInfo(calibration_manager_r->getCameraInfo()));
        else // calibration doesn't exist
        {
            camera_info.reset(new sensor_msgs::CameraInfo());
            camera_info->width = image_r->width;
            camera_info->height = image_r->height;
        }*/

        // WARNING for calibration with cameracalibrator for ROS replace "stereo_rig" with ("/" + camera_name) in both next 2 lines
        image_r->header.frame_id = "stereo_rig";
        //camera_info->header.frame_id = "stereo_rig";
        //camera_info->header.stamp = cv_image.header.stamp;

        mutex_l.lock();
        if(l_imgs_buffer.size() > 0) {
            ros::Time ros_time = ros::Time::now();
            sensor_msgs::Image local_l = l_imgs_buffer.at(l_imgs_buffer.size()-1);
            if(ros_time-local_l.header.stamp < ros::Duration(0.01)) {
                l_imgs_buffer.clear();
                mutex_l.unlock();

                sensor_msgs::CameraInfo::Ptr camera_info_l, camera_info_r;
                camera_info_l.reset(new sensor_msgs::CameraInfo(calibration_manager_l->getCameraInfo()));
                camera_info_r.reset(new sensor_msgs::CameraInfo(calibration_manager_r->getCameraInfo()));
                camera_info_l->header.frame_id = "stereo_rig";
                camera_info_r->header.frame_id = "stereo_rig";
                camera_info_l->header.stamp = ros_time;
                camera_info_r->header.stamp = ros_time;

                image_r->header.frame_id = "stereo_rig";
                image_r->header.stamp = ros_time;
                local_l.header.stamp = ros_time;

                publisher_r.publish(local_l,*camera_info_r);
                publisher_l.publish(image_r,camera_info_l   );
            }
            else {
                mutex_l.unlock();
                mutex_r.lock();
                r_imgs_buffer.clear();
                r_imgs_buffer.push_back(*image_r);
                mutex_r.unlock();
            }

        }
        else {
            mutex_l.unlock();
            mutex_r.lock();
            r_imgs_buffer.clear();
            r_imgs_buffer.push_back(*image_r);
            mutex_r.unlock();
        }

        //publisher_r.publish(image_r,camera_info);
    }

    void configCb_l(photonfocus_camera::photonfocus_monoConfig & config, uint32_t level)
    {
        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera_l->stop();

        camera_l->setDeviceAttribute<PvGenEnum,std::string>("PixelFormat","Mono8");

        //# ----- Image Size Control -----
        camera_l->setDeviceAttribute<PvGenInteger,long>("Width",config.Width*32+768);
        camera_l->setDeviceAttribute<PvGenInteger,long>("Height",config.Height);

        camera_l->setDeviceAttribute<PvGenInteger,long>("OffsetX",config.OffsetX*32);
        camera_l->setDeviceAttribute<PvGenInteger,long>("OffsetY",config.OffsetY);

        //# ----- Exposure and FrameRate -----
        camera_l->setDeviceAttribute<PvGenFloat,double>("ExposureTimeAbs",config.ExposureTimeAbs);
        camera_l->setDeviceAttribute<PvGenBoolean,bool>("ConstantFramerate_CFR",config.ConstantFramerate_CFR);
        if(config.ConstantFramerate_CFR)
            camera_l->setDeviceAttribute<PvGenFloat,double>("Frametime",config.Frametime);

        camera_l->setDeviceAttribute<PvGenBoolean,bool>("Trigger_Interleave",config.Trigger_Interleave);
        if(!config.Trigger_Interleave)\
        {
            camera_l->setDeviceAttribute<PvGenEnum,long>("LinLog_Mode",config.LinLog_Mode);
            if(config.LinLog_Mode == 4)
            {
                std::cout << "UserDefined" << std::endl;
                camera_l->setDeviceAttribute<PvGenInteger,long>("LinLog_Value1",config.LinLog_Value1);
                camera_l->setDeviceAttribute<PvGenInteger,long>("LinLog_Value2",config.LinLog_Value2);
                camera_l->setDeviceAttribute<PvGenInteger,long>("LinLog_Time1",config.LinLog_Time1);
                camera_l->setDeviceAttribute<PvGenInteger,long>("LinLog_Time2",config.LinLog_Time2);
            }
        }
        camera_l->setDeviceAttribute<PvGenInteger,long>("Voltages_BlackLevelOffset",config.Voltages_BlackLevelOffset);

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera_l->start();
    }

    void configCb_r(photonfocus_camera::photonfocus_monoConfig & config, uint32_t level)
    {
        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera_r->stop();

        camera_r->setDeviceAttribute<PvGenEnum,std::string>("PixelFormat","Mono8");

        //# ----- Image Size Control -----
        camera_r->setDeviceAttribute<PvGenInteger,long>("Width",config.Width*32+768);
        camera_r->setDeviceAttribute<PvGenInteger,long>("Height",config.Height);

        camera_r->setDeviceAttribute<PvGenInteger,long>("OffsetX",config.OffsetX*32);
        camera_r->setDeviceAttribute<PvGenInteger,long>("OffsetY",config.OffsetY);

        //# ----- Exposure and FrameRate -----
        camera_r->setDeviceAttribute<PvGenFloat,double>("ExposureTimeAbs",config.ExposureTimeAbs);
        camera_r->setDeviceAttribute<PvGenBoolean,bool>("ConstantFramerate_CFR",config.ConstantFramerate_CFR);
        if(config.ConstantFramerate_CFR)
            camera_r->setDeviceAttribute<PvGenFloat,double>("Frametime",config.Frametime);

        camera_r->setDeviceAttribute<PvGenBoolean,bool>("Trigger_Interleave",config.Trigger_Interleave);
        if(!config.Trigger_Interleave)\
        {
            camera_r->setDeviceAttribute<PvGenEnum,long>("LinLog_Mode",config.LinLog_Mode);
            if(config.LinLog_Mode == 4)
            {
                std::cout << "UserDefined" << std::endl;
                camera_r->setDeviceAttribute<PvGenInteger,long>("LinLog_Value1",config.LinLog_Value1);
                camera_r->setDeviceAttribute<PvGenInteger,long>("LinLog_Value2",config.LinLog_Value2);
                camera_r->setDeviceAttribute<PvGenInteger,long>("LinLog_Time1",config.LinLog_Time1);
                camera_r->setDeviceAttribute<PvGenInteger,long>("LinLog_Time2",config.LinLog_Time2);
            }
        }
        camera_r->setDeviceAttribute<PvGenInteger,long>("Voltages_BlackLevelOffset",config.Voltages_BlackLevelOffset);

        if(level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
            camera_r->start();
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
    ros::NodeHandle nh("~");
    ros::NodeHandle node_handlel("~/left");
    ros::NodeHandle node_handler("~/right");

    signal(SIGINT,signal_handler);

    std::string ip_l;
    std::string ip_r;

    if(!nh.getParam("ip_left",ip_l) || !nh.getParam("ip_right",ip_r))
    {
        std::cout << "Usage: " << argv[0] << " _ip_left:=IP_ADDRESS, _ip_right:=IP_ADDRESS" << std::endl;
        return 0;
    }

    //std::string camera_name = ros::this_node::getName();
    //camera_name = std::string(camera_name.begin()+ros::this_node::getNamespace().length(),camera_name.end());
    std::string camera_name_l = "left";
    std::string camera_name_r = "right";

    boost::shared_ptr<IRALab::PhotonFocusDriver> camera_node(new IRALab::PhotonFocusDriver(camera_name_l, camera_name_r, ip_l, ip_r, node_handlel, node_handler));

    while(ros::ok() && !ros_shutdown)
        ros::spinOnce();

    // the node is shutting down...cleaning
    camera_node.reset();

    ros::shutdown();
    return 0;
}
