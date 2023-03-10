#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "mo_stereo_camera_driver_c.h"

#define MAX_DSP      8192
#define IMAGE_WIDTH  1280
#define IMAGE_HEIGHT 720

unsigned char * GetColorTable(){
  int i;
  unsigned char *colorTable;
  colorTable = (unsigned char *)malloc(3 * 8192 * sizeof(unsigned char));
  colorTable[0] = 0;
  colorTable[1] = 0;
  colorTable[2] = 0;

  for (i = 1; i <= 24; i++) {
    colorTable[i * 3] = 255;
    colorTable[i * 3 + 1] = 255;
    colorTable[i * 3 + 2] = 255;
  }
  for (i = 25; i <= 40; i++) {
    colorTable[i * 3] =
        (int)(255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24));
    colorTable[i * 3 + 1] =
        (int)(255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24));
    colorTable[i * 3 + 2] =
        (int)(255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24));
  }
  for (i = 41; i <= 64; i++) {
    colorTable[i * 3]     = (int)(128 + (int)(5.291668 * (i - 41)));
    colorTable[i * 3 + 1] = (int)(128 - (int)(5.291668 * (i - 41)));
    colorTable[i * 3 + 2] = (int)(128 + (int)(5.291668 * (i - 41)));
  }
  for (i = 65; i <= 120; i++) {
    colorTable[i * 3] = (int)(255 - (int)(4.553571 * (i - 64)));
    colorTable[i * 3 + 1] = 0;
    colorTable[i * 3 + 2] = 255;
  }
  for (i = 121; i <= 176; i++) {
    colorTable[i * 3] = 0;
    colorTable[i * 3 + 1] = (int)(0 + (int)(4.553571 * (i - 120)));
    colorTable[i * 3 + 2] = 255;
  }
  for (i = 177; i <= 320; i++) {
    colorTable[i * 3] = 0;
    colorTable[i * 3 + 1] = 255;
    colorTable[i * 3 + 2] = (int)(255 - (int)(1.770833 * (i - 176)));
  }
  for (i = 321; i <= 800; i++) {
    colorTable[i * 3] = (int)(0 + (int)(0.53125 * (i - 320)));
    colorTable[i * 3 + 1] = 255;
    colorTable[i * 3 + 2] = 0;
  }
  for (i = 801; (i <= 2048); i++) {
    colorTable[i * 3] = 255;
    colorTable[i * 3 + 1] = (int)(255 - (int)(0.204327 * (i - 800)));
    colorTable[i * 3 + 2] = 0;
  }
  for (i = 2049; i < 8192; i++) {
    colorTable[i * 3] = 255;
    colorTable[i * 3 + 1] = 0;
    colorTable[i * 3 + 2] = 0;
  }
  return colorTable;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mo_rgbd");

    ros::NodeHandle nh;
    //basic param
    int image_half_height = IMAGE_HEIGHT>>1;
    int image_half_width = IMAGE_WIDTH>>1;
    // roslaunch params
    std::string camPath;
    std::string topicName;
    int fps_;
    int PCLOUD_REDUCE_RATE;

    nh.param<std::string>("cam_path",camPath,"/dev/video0");
    nh.param<std::string>("topic_name",topicName,"/pointcloud_1");
    nh.param<int>("fps",fps_,30);
    nh.param<int>("point_cloud_down_sample",PCLOUD_REDUCE_RATE, 1);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_pub = it.advertise("mo_cam/image_color", 1);
    image_transport::Publisher depth_pub = it.advertise("mo_cam/image_depth", 1);
    image_transport::Publisher dspColor_pub = it.advertise("mo_cam/image_dsp",1);
    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("mo_cam/camera_info", 1);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topicName, 1);
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    sensor_msgs::PointCloud2 output;

    //get camera info
    camera_info_manager::CameraInfoManager cam_info_manager(nh, "mo_cam/camera_info","package://moak_camera/config/calib.yaml");
    sensor_msgs::CameraInfo cam_info = cam_info_manager.getCameraInfo();

    int32_t          s32Result     = 0;
    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;
    
    std::cout << "the version of SDK   : " << moGetSdkVersion() << std::endl;
    std::cout << "the path    of camera: " << camPath.data()    << std::endl;
        
    /**< 1. Open specific camera */
    s32Result = moOpenUVCCameraByPath(camPath.c_str(), &hCameraHandle);
    if (0 != s32Result) {
        printf("Error: moOpenUVCCameraByPath return %d\n", s32Result);
        return -1;
    }
    
    /**< 2. set RGBD mode : frame = Disparity + YUV_I420 */
  //  s32Result = moSetVideoMode(hCameraHandle, MVM_RGBD_DENSE);
     s32Result = moSetVideoMode(hCameraHandle, MVM_RGBD);
   if (0 != s32Result) {
       printf("Error: moSetVideoMode return %d\n", s32Result);
       return -2;
   }
    
    /**< 3. Get the Bxf and Base of camera */
    float fBxf  = 0.0f;
    float fBase = 0.0f;
    s32Result = moGetBxfAndBase(hCameraHandle,
                                &fBxf,
                                &fBase);
    if (0 != s32Result) {
        printf("Error: moGetBxfAndBase return %d\n", s32Result);
        return -3;
    }
    std::cout << "fBF: " << fBxf << ", fBase: " << fBase << std::endl;
    
    // generate tables
    float* m_mapDspToDisZ    = new float[MAX_DSP];
    float* m_mapDspToDisZ_M  = new float[MAX_DSP];
    float* m_mapDspToDisXY_M = new float[MAX_DSP];
    unsigned char* colorTable;

    for(int i=1; i<MAX_DSP; i++){
        m_mapDspToDisZ[i]    = fBxf * 32.0 / i;
        m_mapDspToDisZ_M[i]  = fBxf * 32.0 / i / 1000.0;
        m_mapDspToDisXY_M[i] = fBase *32.0 / i / 1000.0;
    }
    colorTable = GetColorTable();

    // for(int i=1; i<MAX_DSP; i++){
    //     std::cout << "dsp: " << i << ", disz: " << m_mapDspToDisZ_M[i] << std::endl;
    // }

    cloud.width = IMAGE_WIDTH;
    cloud.height = IMAGE_HEIGHT;
    cloud.points.resize(cloud.width * cloud.height);
    
    uint64_t  u64ImageFrameNum      = 0;
    uint8_t*  pu8FrameBuffer        = NULL;
    uint16_t* pu16RGBDDisparityData = NULL;
    uint8_t*  pu8RGBDYUVI420Img     = NULL;

    ros::Rate loop_rate(fps_);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        
        /**< 4. get current video frame */
        s32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != s32Result) {
            printf("Error: moGetCurrentFrame return %d\n", s32Result);
            return -4;
        }
        std::cout << "Frame Number: " << u64ImageFrameNum << std::endl;

        /**< 5. get left and right image */
        s32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);
        if (0 != s32Result) {
            printf("Error: moGetRGBDImage return %d\n", s32Result);
            return -5;
        }
                
        // Fill in the cloud data
        cloud.clear();
        int nCount = 0;
        
        unsigned short *pDspHead = pu16RGBDDisparityData;
        
        for(int y = 0;y<IMAGE_HEIGHT;y++){
            for(int x = 0;x<IMAGE_WIDTH;x++,pDspHead ++){
                if((*pDspHead > 0)&&(x%PCLOUD_REDUCE_RATE==0)&&(y%PCLOUD_REDUCE_RATE==0)){
                    nCount ++;
                }
            }
        }
        cloud.width = nCount;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);
        printf("%d \n",nCount);

        cv::Mat yuvImg(IMAGE_HEIGHT * 1.5, IMAGE_WIDTH,CV_8UC1, pu8RGBDYUVI420Img);
        cv::Mat rgbImg(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3);
        cv::Mat depth_img(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);
        cv::Mat dsp_color_img = cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2BGR_I420);

        // depth image and cloud point
        nCount = 0;
        pDspHead = pu16RGBDDisparityData;
        unsigned char *pRgbHead = rgbImg.data;

        for(int y = 0;y<IMAGE_HEIGHT;y++){
            unsigned short *pdepth = depth_img.ptr<unsigned short>(y);
            uchar* pImgData = dsp_color_img.data + y*dsp_color_img.step;

            for(int x = 0;x<IMAGE_WIDTH;x++,pDspHead ++, pdepth++, pRgbHead +=3){
                unsigned short usd = *pDspHead;

                if(usd > 0){
                    // point cloud downsample
                    if((x%PCLOUD_REDUCE_RATE==0)&&(y%PCLOUD_REDUCE_RATE==0)){
                        cloud.points[nCount].x = m_mapDspToDisZ_M[usd];
                        cloud.points[nCount].y = m_mapDspToDisXY_M[usd]*(image_half_width-x);
                        cloud.points[nCount].z = m_mapDspToDisXY_M[usd]*(image_half_height-y);
                        cloud.points[nCount].b = pRgbHead[0];
                        cloud.points[nCount].g = pRgbHead[1];
                        cloud.points[nCount].r = pRgbHead[2];
                        cloud.points[nCount].a = 255;
                        nCount ++;
                    }
                    // no downsample for depth image and dsp-color
                    *pdepth = (unsigned short) (m_mapDspToDisZ[usd]);

                    *(pImgData++) = colorTable[usd * 3 + 2];
                    *(pImgData++) = colorTable[usd * 3 + 1];
                    *(pImgData++) = colorTable[usd * 3];
                }else{
                    *pdepth = static_cast<unsigned short> (0);

                    *(pImgData++) = 0;
                    *(pImgData++) = 0;
                    *(pImgData++) = 0;
                }
            }
        }
        //Convert the cloud and image to ROS message
        std_msgs::Header header;
        header.stamp = now;
        header.frame_id = "camera_front";
        // header.frame_id = "base_link";

        pcl::toROSMsg(cloud, output);
        cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, "bgr8", rgbImg);
        cv_bridge::CvImage cv_depth = cv_bridge::CvImage(header, "mono16", depth_img); //mono16
        cv_bridge::CvImage cv_dsp_color = cv_bridge::CvImage(header, "bgr8", dsp_color_img);
        
        output.header.frame_id   = "camera_front";
        cam_info.header.frame_id = "camera_front";
        // output.header.frame_id   = "base_link";
        // cam_info.header.frame_id = "base_link";
        
        output.header.stamp = now;
        cam_info.header.stamp = now;
        
        pcl_pub.publish(output);
        cam_info_pub.publish(cam_info);
        img_pub.publish(cv_image.toImageMsg());
        depth_pub.publish(cv_depth.toImageMsg());
        dspColor_pub.publish(cv_dsp_color.toImageMsg());
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    free(m_mapDspToDisZ);
    free(m_mapDspToDisZ_M);
    free(m_mapDspToDisXY_M);
    free(colorTable);

    /**< 6. Close specific camera */
    moCloseCamera(&hCameraHandle);
    
    return 0;
}
