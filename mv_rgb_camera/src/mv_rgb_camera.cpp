#include "CameraApi.h"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "iostream"
#include <boost/array.hpp>
#include <vector>
#include <boost/range/algorithm.hpp>

// =======================================================================

void set_color_msg(sensor_msgs::ImagePtr color_msg, int h, int w, int c);
void set_camera_info(ros::NodeHandle& node, sensor_msgs::CameraInfoPtr msg, int h, int w);

// =======================================================================

int main(int argc, char** argv) {
    int                     iCameraCounts = 1; // Nuber of Camera.
    int                     iStatus=-1; // Camera status, Normal state => 0, Err => else one.
    tSdkCameraDevInfo       tCameraEnumList; // Connected Camera device lists.
    int                     hCamera; // Selected Camera device.
    tSdkCameraCapbility     tCapability; // Structure for save Camera Infomations.
    tSdkFrameHead           sFrameInfo; // Structure for save Camera Frame Infomations.
    BYTE*			        pbyBuffer; // Buffer for saving Camera Frame data.
    int                     channel=3; // camera channels
    unsigned char           *g_pRgbBuffer; // Buffer for saving image data.

    int                     image_width = 4096;
    int                     image_height = 3000;
    int                     image_fps = 23;

// =======================================================================

    ROS_INFO("[INIT ROS] : Initializing ROS Started...");
    ros::init(argc, argv, "mv_rgb_camera");
    ros::NodeHandle node("~");
    ROS_INFO("[INIT ROS] : Done..");

    CameraSdkInit(1);

    // Find Device List & caculate amount
    // [return] :  if found camera => 0, else => -16?
    ROS_INFO("[INIT CAMERA]: Searching camera Devices...");

    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts); 
    if(iCameraCounts==0 || iStatus){
        ROS_FATAL("[INIT CAMERA]: Cannot Find Camera Device. Exit.");
        return -1;
    }
    ROS_INFO("[INIT CAMERA]: Found %d of Devices.", iCameraCounts);

    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    if (iStatus) {
        ROS_INFO("[INIT CAMERA]: Getting camera device is Failed. Exit.");
        return -1;
    }
    ROS_INFO("[INIT CAMERA]: Getting camera device Complete.");

    // Set image encoding both MONO8 or BRG8.
    if(tCapability.sIspCapacity.bMonoSensor){
        ROS_WARN("[INIT CAMERA]: Camera sensor => MONO. Streaing Monogray Image.");
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        ROS_WARN("[INIT CAMERA]: Camera sensor => COLOR. Streaing Color Image.");
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    ROS_INFO("[INIT CAMERA]: Done.");

// =======================================================================

    ROS_INFO("[SET PARAM]: Parameter Setting Started...");
    // get parameters.
    node.param<int>("image_width", image_width, image_width);
    node.param<int>("image_height", image_height, image_height);
    node.param<int>("fps", image_fps, image_fps);

    // Set Image stream's width, height, fps.
    tSdkImageResolution sImageSize;
    memset(&sImageSize,0,sizeof(tSdkImageResolution));
    sImageSize.iIndex=0xff;
    sImageSize.iHOffsetFOV=0;
    sImageSize.iVOffsetFOV=0;
    sImageSize.iWidthFOV=image_width;
    sImageSize.iHeightFOV=image_height;
    sImageSize.iWidth=image_width;
    sImageSize.iHeight=image_height;
    CameraSetImageResolution(hCamera,&sImageSize);
    CameraSetFrameRate(hCamera, image_fps);

    ROS_WARN("[SET PARAM]: Done. Image Resolution => (%d, %d) / %d fps", image_width, image_height, image_fps);

    // Get camera informations and store to tCapability.
    CameraGetCapability(hCamera,&tCapability);
    // Set image data Buffer size.
    g_pRgbBuffer = (unsigned char*)malloc(
        sImageSize.iHeight *
        sImageSize.iWidth *
        channel
    );

// =======================================================================

    ROS_INFO("[SET PUBLISHER]: Publisher Setting Stared...");

    // Set Image Transport Publisher.
    image_transport::ImageTransport it(node);
    image_transport::Publisher it_pub = it.advertise("/rdv_mv_rgb_0001/color/image_rect_raw", 10);
    boost::shared_ptr<sensor_msgs::Image> color_msg(new sensor_msgs::Image);
    set_color_msg(color_msg, sImageSize.iHeight, sImageSize.iWidth, channel);

    // Set camera info message using "config/mv_rgb_camera.yaml" file.
    ros::Publisher cam_pub = node.advertise<sensor_msgs::CameraInfo>("/rdv_mv_rgb_0001/color/camera_info", 1000);
    boost::shared_ptr<sensor_msgs::CameraInfo> camera_info_msg(new sensor_msgs::CameraInfo);
    set_camera_info(node, camera_info_msg, sImageSize.iHeight, sImageSize.iWidth);

    ROS_INFO("[SET PUBLISHER]: Done.");

    // Trigger for starting camera.
    CameraPlay(hCamera);
    ROS_INFO("[MAIN]: Now on camera streaming started.");
    
// ===================================================================

    while (ros::ok()) {
        // Set pbyBuffer pointer to save camera streaming data.
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
            // get image data from pbyBuffer to g_pRgbBuffer.
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            ros::Time cur = ros::Time::now();

            color_msg->header.stamp = cur;
            std::memcpy (
                &color_msg->data[0],
                g_pRgbBuffer,
                color_msg->step * color_msg->height
            );
            it_pub.publish(color_msg);

            camera_info_msg->header.stamp = cur;
            cam_pub.publish(camera_info_msg);
		    
            // // View Camera data with opencv.
		    // cv::Mat matImage(
			// 		cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight), 
			// 		sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
			// 		g_pRgbBuffer
			// 		);
            // matImage.reshape(640, 480);
			// imshow("Opencv Demo", matImage);
            // cv::waitKey(5);

			CameraReleaseImageBuffer(hCamera, pbyBuffer);
		}
    }

    CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    return 0;
}

// =======================================================================

void set_color_msg(sensor_msgs::ImagePtr color_msg, int h, int w, int c) {
    color_msg->height = h;
    color_msg->width = w;
    color_msg->header.frame_id = "mv_frame";
    color_msg->encoding = c ==3? "rgb8" : "mono8";
    color_msg->is_bigendian = false;
    color_msg->step = w * c;
    color_msg->data.resize(h * w * c);
}

void set_camera_info(ros::NodeHandle& node, sensor_msgs::CameraInfoPtr msg, int h, int w) {
    std::string model;
    std::vector<double> D, K, R, P;
    node.param<std::string>("/distortion_model", model, "plumb_bob");
    node.param<std::vector<double>>("/distortion_coefficients/data", D, {0});
    node.param<std::vector<double>>("/camera_matrix/data", K, {0});
    node.param<std::vector<double>>("/rectification_matrix/data", R, {0});
    node.param<std::vector<double>>("/projection_matrix/data", P, {0});

    msg->header.frame_id = "mv_frame";
    msg->height = h;
    msg->width = w;
    msg->distortion_model = "plumb_bob";
    msg->D.assign(D.begin(), D.end());
    boost::range::copy(K, msg->K.begin());
    boost::range::copy(R, msg->R.begin());
    boost::range::copy(P, msg->P.begin());
}