#include "ArenaApi.h"

#include <vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


#define CONNECT_TIMEOUT 1000

// =======================================================================

// Ros variables.
ros::Publisher pcl_pub;
boost::shared_ptr<sensor_msgs::PointCloud2> pcl_msg(new sensor_msgs::PointCloud2);
ros::Publisher intensity_pub;
boost::shared_ptr<sensor_msgs::Image> intensity_msg(new sensor_msgs::Image);
ros::Publisher cam_pub;
boost::shared_ptr<sensor_msgs::CameraInfo> camera_info_msg(new sensor_msgs::CameraInfo);

float* pcl_data;
uint16_t* intensity_data;

// Areana Key variables.
Arena::ISystem* pSystem; // variable for camera sysyem
Arena::IDevice* pDevice; // variable for device management
GenICam::gcstring SN;

GenICam::gcstring operatingModeInitial = "";
GenICam::gcstring exposureTimeInitial = "";
GenICam::gcstring conversionGainInitial = "";
int64_t imageAccumulationInitial = 0;
bool spatialFilterInitial;
bool confidenceThresholdInitial;

// Varaiables for 3D Coordinate ==> real distance.
float scaleX;
float offsetX;
double scaleY;
float offsetY;
double scaleZ;

// Lock for protect PCL & intensity data.
std::mutex lock;

void set_pcl_msg(sensor_msgs::PointCloud2Ptr msg);
void set_intensity_msg(sensor_msgs::ImagePtr msg);
void set_camera_info(Arena::IDevice* pDevice, sensor_msgs::CameraInfoPtr msg);

void init_cam(ros::NodeHandle &node);
void init_publisher(ros::NodeHandle &node);
void signal_handler(int sig);

// =======================================================================

// Image Callback class.
class ImageCallback : public Arena::IImageCallback {
public:
    ImageCallback() {
    }
    ~ImageCallback() {}

    static void send_msgs(Arena::IImage* pImage) {
        pcl_msg->header.stamp = camera_info_msg->header.stamp = intensity_msg->header.stamp = ros::Time::now();

        const uint8_t *pInput = pImage->GetData();
        const uint8_t *pIn = pInput;

        lock.lock();
        {
            for (int i=0; i < 640*480; i++) {
                uint16_t x_raw = *reinterpret_cast<const uint16_t*>(pIn);
                uint16_t y_raw = *reinterpret_cast<const uint16_t*>((pIn+2));
                uint16_t z_raw = *reinterpret_cast<const uint16_t*>((pIn+4));
                uint16_t i_raw = *reinterpret_cast<const uint16_t*>((pIn+6));

                pcl_data[4*i] = ((float)x_raw * scaleX + offsetX) * 0.001f;
                pcl_data[4*i + 1] = ((float)y_raw * scaleY + offsetY) * 0.001f;
                pcl_data[4*i + 2] = (z_raw == 0) ? std::numeric_limits<float>::quiet_NaN () : ((float)z_raw * scaleZ) * 0.001f;
                pcl_data[4*i + 3] = (float)i_raw;
                intensity_data[i] = i_raw;

                pIn += 8;
            }

            pcl_pub.publish(pcl_msg);
            intensity_pub.publish(intensity_msg);
            cam_pub.publish(camera_info_msg);
        }
        lock.unlock();
    }

    void OnImage(Arena::IImage* pImage) {
        std::thread t(send_msgs, pImage);
        t.join();
    }
};

// Disconnection Callback. Only using for notificate Disconnection once.
class DisconnectionCallback : public Arena::IDisconnectCallback
{
public:
	DisconnectionCallback() {};
	virtual ~DisconnectionCallback() {};

	void OnDeviceDisconnected(Arena::IDevice* pDevice)
	{
        ROS_FATAL("Disconnection Occurred.");
	}
};

// =======================================================================

// main.
int main(int argc, char** argv) {
    // Initializing ROS.
    ros::init(argc, argv, "helios_camera", ros::init_options::NoSigintHandler);
    ros::NodeHandle node("~");

    // Create Signal handler.
    signal(SIGINT, signal_handler);

	ROS_INFO("[HELIOS 2+] : Start Progress.");

    try 
    {
        // Initializing Arena Camera Options.
        init_cam(node);
        ROS_INFO("[INIT] : Init Complete.");

        // Set ros publisher & message.
        init_publisher(node);
        ROS_INFO("[ROS] : Initializing ros publisher complete.");

        // Starting Camera stream.
        pDevice->StartStream(5);
        ROS_INFO("[MAIN] : Camera Stream Started.");

    }
    catch (GenICam::GenericException& ge)
	{
        ROS_FATAL("GenICam exception thrown:\n %s", ge.what());
	}
	catch (std::exception& ex)
	{
        ROS_FATAL("Standard exception thrown:\n%s", ex.what());
	}
	catch (...)
	{
		ROS_FATAL("Unknown Exception Occured.");
	}

    ros::spin();

    return 0;
}

// =======================================================================

void init_cam(ros::NodeHandle &node) {
    ROS_INFO("[INIT] : Start Init.");
    pSystem = Arena::OpenSystem();

    // Find devices during 3 Sec.
    ROS_INFO("[INIT] : Find Camera devices during 3 second...");
    pSystem->UpdateDevices(CONNECT_TIMEOUT);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();

    // Case of cannot find any camera. Exit.
    if (deviceInfos.size() == 0)
    {
        ROS_FATAL("[INIT] : Cannot find Helios camera.\n\nPress enter to exit.");
        std::getchar();
        raise(SIGUSR1);
    }

    // Set Using device to first ine.
    pDevice = pSystem->CreateDevice(deviceInfos[0]);
    SN = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceSerialNumber");
    ROS_INFO("[INIT]: Using Camera device[0] => SN : %s", SN.c_str());

    // save Camera setting.
    operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode");
    exposureTimeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector");
    conversionGainInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain");
    imageAccumulationInitial = Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation");
    spatialFilterInitial = Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable");
    confidenceThresholdInitial = Arena::GetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable");

    // Upload Camera setting.
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateA");
    scaleX = static_cast<float>(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale"));
	offsetX = static_cast<float>(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateB");
	scaleY = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale");
	offsetY = static_cast<float>(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dCoordinateSelector", "CoordinateC");
	scaleZ = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "Scan3dCoordinateScale");

    ROS_INFO("[INIT]: Loading camera setting done.");

    // Camera Setting for Connections.
    Arena::SetNodeValue<GenICam::gcstring> (pDevice->GetNodeMap(),
        "AcquisitionMode",
        "Continuous"
    );
    Arena::SetNodeValue<GenICam::gcstring> (pDevice->GetTLStreamNodeMap(),
        "StreamBufferHandlingMode",
        "NewestOnly"
    );
    Arena::SetNodeValue<bool> (pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

    // Camera Setting for Smoothing depth image.
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", node.param<std::string>("mode", "Distance3000mmSingleFreq").c_str());
    ROS_INFO("[INIT] : Scan3d Operating Mode Set to => %s", Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode").c_str());
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", node.param<std::string>("exp_time", "Exp1000Us").c_str());
    ROS_INFO("[INIT] : Exposure time Set to => %s", Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector").c_str());
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", node.param<std::string>("gain", "Low").c_str());
    ROS_INFO("[INIT] : Conversion Gain Set to => %s", Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain").c_str());
    Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", (int64_t)node.param<int>("accumulation", 2));
    ROS_INFO("[INIT] : Image Accumulation Set to => %ld", Arena::GetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation"));
    Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", true);
    Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", true);

    // Initializing Callback.
    ImageCallback* callback = new ImageCallback();
    DisconnectionCallback* dis_cb = new DisconnectionCallback;
    pDevice->RegisterImageCallback(callback);
    pSystem->RegisterDeviceDisconnectCallback(pDevice, dis_cb);


    // Set image format to CY16. ==> depth data & intensity data using 16 bits per pixel
    Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");
}

void init_publisher(ros::NodeHandle &node) {
    // Initializing Point Cloud Publisher & Message.
    pcl_pub = node.advertise<sensor_msgs::PointCloud2>("/rdv_helios_0001/depth/point", 10);
    set_pcl_msg(pcl_msg);

    // Initializing Intensity Image Publisher & Message.
    intensity_pub = node.advertise<sensor_msgs::Image>("/rdv_helios_0001/depth/intensity_raw", 10);
    set_intensity_msg(intensity_msg);

    // Initializing CameraInfo Publisher & Message.
    cam_pub = node.advertise<sensor_msgs::CameraInfo>("/rdv_helios_0001/depth/camera_info", 1000);
    set_camera_info(pDevice, camera_info_msg);
}

// Setting for Point Cloud Message.
void set_pcl_msg(sensor_msgs::PointCloud2Ptr msg) {
    msg->header.frame_id = "helios_frame";
    msg->height = 480;
    msg->width = 640;
    msg->is_bigendian = false;
    msg->is_dense = true;
    msg->point_step = 16;
    msg->row_step = 16 * 640;
    msg->data.resize(16 * 640 * 480);
    pcl_data = reinterpret_cast<float *>(&pcl_msg->data[0]);

    msg->fields.resize(4);

    msg->fields[0].name = "x";
    msg->fields[1].name = "y";
    msg->fields[2].name = "z";
    msg->fields[3].name = "intensity";

    for (int i=0; i < 3; i++) {
        msg->fields[i].count = 1;
        msg->fields[i].datatype = 7;
        msg->fields[i].offset = i*4;
    }
}

// Setting for Intensity Image Message.
void set_intensity_msg(sensor_msgs::ImagePtr msg) {
    msg->header.frame_id = "helios_frame";
    msg->encoding = "16UC1";
    msg->height = 480;
    msg->width = 640;
    msg->is_bigendian = false;
    msg->step = 640 * 2;
    msg->data.resize(2 * 640*480);

    intensity_data = reinterpret_cast<uint16_t *>(&intensity_msg->data[0]);
}

void set_camera_info(Arena::IDevice* pDevice, sensor_msgs::CameraInfoPtr msg) {
    msg->header.frame_id = "helios_frame";
    msg->height = 480;
    msg->width = 640;
    msg->distortion_model = "plumb_bob";

    double fx = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibFocalLengthX");
    double fy = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibFocalLengthY");
    double px = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibOpticalCenterX");
    double py = Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibOpticalCenterY");

    for (int i=0; i<5; i++) {
        GenICam::gcstring tmp = ("Value"+std::to_string(i)).c_str();
        Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "CalibLensDistortionValueSelector", tmp);
        msg->D.push_back(Arena::GetNodeValue<double>(pDevice->GetNodeMap(), "CalibLensDistortionValue"));
    }
    
    msg->K = {
        fx, 0.0, px,
        0.0, fy, py,
        0.0, 0.0, 1.0
    };
    msg->R = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    msg->P = {
        fx, 0.0, px, 0.0,
        0.0, fy, py, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
}

// =======================================================================

// Signal Event handler.
void signal_handler(int sig) {
    ROS_FATAL("[EXIT] : Starting progress.");

    try {
        if (pDevice->IsConnected()) {
            pDevice->StopStream();
            ROS_WARN("[EXIT] : Camera Connection detected. Now on Connection closed.");
        }
        pDevice->DeregisterAllImageCallbacks();
        pSystem->DeregisterAllDeviceDisconnectCallbacks();

        if (operatingModeInitial != ""){
            Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dConfidenceThresholdEnable", confidenceThresholdInitial);
            Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "Scan3dSpatialFilterEnable", spatialFilterInitial);
            Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", imageAccumulationInitial);
            Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ConversionGain", conversionGainInitial);
            Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "ExposureTimeSelector", exposureTimeInitial);
            Arena::SetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "Scan3dOperatingMode", operatingModeInitial);
            ROS_WARN("[EXIT] : camera settings Reinitialized.");
        }
    

        pSystem->DestroyDevice(pDevice);
        Arena::CloseSystem(pSystem);
        ROS_WARN("[EXIT] : Arena variables Destroied.");
    }
    catch (GenICam::GenericException& ge) {
        ROS_WARN("GenI Exception:\n%s", ge.what());
        throw;
    }

    ROS_WARN("[EXIT] : All of progress done. EXIT.");
    ros::shutdown();
}