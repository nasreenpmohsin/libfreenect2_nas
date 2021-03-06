/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */





#include "functions_kinect.h"
//#include "functions_kinect.cpp"
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace boost;
//using namespace func_kinect;


std::string  extremaStr[K_EX]={"ex1","ex2","ex3","ex4","ex5"};

// Initialize vector with strings using push_back
// command


bool protonect_shutdown = false;

void sigint_handler(int s)
{
    protonect_shutdown = true;
}
/*loat distance(pcl::PointXYZ a, pcl::PointXYZ b) {
  return sqrt((a.x-b.x)*(a.x-b.x)
             +(a.y-b.y)*(a.y-b.y)
             +(a.z-b.z)*(a.z-b.z));
}
float distance2D(Point2D a, Point2D b){
    return sqrt((a.x-b.x)*(a.x-b.x)
                +(a.y-b.y)*(a.y-b.y));
}*/

int main(int argc, char *argv[])
{
    // File Storage

    std::string set = "set57/";
    std::cout << "please enter the set name (eg: set50_eval/ ): ";
    std::cin >> set;
    // declaring folder and filename

    std::string foldername = "/home/nmohsin/cal_images/" ;

    foldername=foldername+set;
    //declaring timestamp
    struct timeval start, end;

    long mtime, seconds, useconds;
// declaring file count
    int count = 0;
    int data_count = 0;
    std::ofstream myfile;

//initiate
    std::string program_path(argv[0]);
    size_t executable_name_idx = program_path.rfind("Protonect");

    std::string binpath = "/";

    if(executable_name_idx != std::string::npos)
    {
        binpath = program_path.substr(0, executable_name_idx);
    }

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    //**********************//
    libfreenect2::Freenect2 freenect2_1;
    libfreenect2::Freenect2Device *dev1 = 0;
    libfreenect2::PacketPipeline *pipeline1 = 0;
//***************************//
    libfreenect2::Freenect2 freenect2_2;
    libfreenect2::Freenect2Device *dev2 = 0;
    libfreenect2::PacketPipeline *pipeline2 = 0;
    //Detecting devices
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    //std::string serial = freenect2.getDefaultDeviceSerialNumber();

    std::string serial = freenect2.getDeviceSerialNumber(0);
    std::string serial1 = freenect2.getDeviceSerialNumber(1);
    std::string serial2 = freenect2.getDeviceSerialNumber(2);
    bool record = false;
    for(int argI = 1; argI < argc; ++argI)
    {
        const std::string arg(argv[argI]);
        if(arg == "r") record=true;
        else if(arg == "cpu")
        {
            if(!pipeline)
                pipeline = new libfreenect2::CpuPacketPipeline();
            if(!pipeline1)
                pipeline1 = new libfreenect2::CpuPacketPipeline();
            if(!pipeline2)
                pipeline2 = new libfreenect2::CpuPacketPipeline();
        }
        else if(arg == "gl")
        {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenGLPacketPipeline();
            if(!pipeline1)
                pipeline1 = new libfreenect2::OpenGLPacketPipeline();
            if(!pipeline2)
                pipeline2 = new libfreenect2::OpenGLPacketPipeline();
#else
            std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cl")
        {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
            if(!pipeline1)
                pipeline1 = new libfreenect2::OpenCLPacketPipeline(-1);
            if(!pipeline2)
                pipeline2 = new libfreenect2::OpenCLPacketPipeline(-1);
#else
            std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
        {
            serial = arg[0];
            serial1 = arg[1];
            serial2 = arg[2];
        }
        else
        {
            std::cout << "Unknown argument: " << arg << std::endl;
        }
    }
    //Opening serial channel from 3 Devices///////
    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }
    if(pipeline1)
    {
        dev1 = freenect2.openDevice(serial1, pipeline1);
    }
    else
    {
        dev1 = freenect2.openDevice(serial1);
    }
    if(pipeline2)
    {
        dev2 = freenect2.openDevice(serial2, pipeline2);
    }
    else
    {
        dev2 = freenect2.openDevice(serial2);
    }
/****************************/

    if(dev == 0)
    {
        std::cout << "failure opening device 1!" << std::endl;
        return -1;
    }

    if(dev1 == 0)
    {
        std::cout << "failure opening device 2!" << std::endl;
        return -1;
    }
    if(dev2 == 0)
    {
        std::cout << "failure opening device 3!" << std::endl;
        return -1;
    }

    signal(SIGINT,sigint_handler);

    protonect_shutdown = false;
///////////Device 1
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    std::cout << "device serial 1: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware 1: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    //Intrinsic parameters
    libfreenect2::Freenect2Device::IrCameraParams G;
    G=dev->getIrCameraParams();
    float fx=G.fx;
    float fy=G.fy;
    float cx=G.cx;
    float cy=G.cy;
    std::cout << "Sensor 1 parameters: " << fx << " " << fy <<" "<< cx <<" "<< cy << std::endl;
    libfreenect2::Freenect2Device::Config config;
    config.MinDepth = 0.5f;
    config.MaxDepth = 8.0f;
    dev->setConfiguration(config);


/////////// Device 2
    libfreenect2::SyncMultiFrameListener listener1(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames1;
    libfreenect2::Frame undistorted1(512, 424, 4), registered1(512, 424, 4);

    dev1->setColorFrameListener(&listener1);
    dev1->setIrAndDepthFrameListener(&listener1);
    dev1->start();

    std::cout << "device serial 2: " << dev1->getSerialNumber() << std::endl;
    std::cout << "device firmware 2: " << dev1->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration1 = new libfreenect2::Registration(dev1->getIrCameraParams(), dev1->getColorCameraParams());
    //Intrinsic parameters
    libfreenect2::Freenect2Device::IrCameraParams G1;
    G1=dev1->getIrCameraParams();
    float fx1=G1.fx;
    float fy1=G1.fy;
    float cx1=G1.cx;
    float cy1=G1.cy;
    std::cout << "Sensor 2 parameters: " << fx1 << " " << fy1 <<" "<< cx1 <<" "<< cy1 << std::endl;
    dev1->setConfiguration(config);

///////////Device 3
    libfreenect2::SyncMultiFrameListener listener2(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames2;
    libfreenect2::Frame undistorted2(512, 424, 4), registered2(512, 424, 4);

    dev2->setColorFrameListener(&listener2);
    dev2->setIrAndDepthFrameListener(&listener2);
    dev2->start();

    std::cout << "device serial 3: " << dev2->getSerialNumber() << std::endl;
    std::cout << "device firmware 3: " << dev2->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration2 = new libfreenect2::Registration(dev2->getIrCameraParams(), dev2->getColorCameraParams());
    //Intrinsic parameters
    libfreenect2::Freenect2Device::IrCameraParams G2;
    G2=dev2->getIrCameraParams();
    float fx2=G2.fx;
    float fy2=G2.fy;
    float cx2=G2.cx;
    float cy2=G2.cy;
    std::cout << "Sensor 3 parameters: " << fx2 << " " << fy2 <<" "<< cx2 <<" "<< cy2 << std::endl;
    dev2->setConfiguration(config);

/***********************************************************************/

    //Initializing Mask
    cv::Mat fgMaskMOG2_1;
    cv::Ptr< cv::BackgroundSubtractor> pMOG2_1;
    pMOG2_1 = cv::createBackgroundSubtractorMOG2(20,16,false);
    cv::Mat fgMaskMOG2_2;
    cv::Ptr< cv::BackgroundSubtractor> pMOG2_2;
    pMOG2_2 = cv::createBackgroundSubtractorMOG2(20,16,false);
    cv::Mat fgMaskMOG2_3;
    cv::Ptr< cv::BackgroundSubtractor> pMOG2_3;
    pMOG2_3 = cv::createBackgroundSubtractorMOG2(20,16,false);
//    cv::viz::Viz3d viewer, viewer1, viewer2, viewer3,viewer4;
    cv::Mat cloudMat, cloudMat1, cloudMat2;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr(new pcl::visualization::PCLVisualizer("3d Viewer"));

    //3D visualization from pcl library//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p1(new pcl::visualization::PCLVisualizer("3d Viewer1"));

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr(new pcl::visualization::PCLVisualizer("3d Viewer"));

    while(!protonect_shutdown &  !viewer_p1->wasStopped()){

        //gettimeofday(&start, NULL);
        listener.waitForNewFrame(frames);
        listener1.waitForNewFrame(frames1);
        listener2.waitForNewFrame(frames2);

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        libfreenect2::Frame *rgb1 = frames1[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir1 = frames1[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth1 = frames1[libfreenect2::Frame::Depth];

        libfreenect2::Frame *rgb2 = frames2[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir2 = frames2[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth2 = frames2[libfreenect2::Frame::Depth];
        //gettimeofday(&end, NULL);

        registration->apply(rgb,depth,&undistorted,&registered);
        registration1->apply(rgb1,depth1,&undistorted1,&registered1);
        registration2->apply(rgb2,depth2,&undistorted2,&registered2);

        ///
        count++;

        cv::Mat rgb_1;
        cv::Mat rgb1_1;
        cv::Mat rgb2_1;
        const int depthWidth=512;//depth->width;
        const int depthHeight=424;//depth->height;
        //cv::Mat depim = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
        //cv::flip(depim,depim,1);
        cv::Mat MatUndist = cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        cv::flip(MatUndist,MatUndist,1);
        cv::Mat depim=MatUndist;
        cv::Mat irim = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data);
        cv::flip(irim,irim,1);
        cv::Mat MatReg = cv::Mat(registered.height, registered.width, CV_8UC4, registered.data);
        cv::flip(MatReg,MatReg,1);

        //cv::Mat depim1 = cv::Mat(depth1->height, depth1->width, CV_32FC1, depth1->data);
        //cv::flip(depim1,depim1,1);
        cv::Mat MatUndist1 = cv::Mat(undistorted1.height, undistorted1.width, CV_32FC1, undistorted1.data);
        cv::flip(MatUndist1,MatUndist1,1);
        cv::Mat depim1=MatUndist1;
        cv::Mat irim1 = cv::Mat(ir1->height, ir1->width, CV_32FC1, ir1->data);
        cv::flip(irim1,irim1,1);
        cv::Mat MatReg1 = cv::Mat(registered1.height, registered1.width, CV_8UC4, registered1.data);
        cv::flip(MatReg1,MatReg1,1);

        //cv::Mat depim2 = cv::Mat(depth2->height, depth2->width, CV_32FC1, depth2->data);
        // cv::flip(depim2,depim2,1);
        cv::Mat MatUndist2 = cv::Mat(undistorted2.height, undistorted2.width, CV_32FC1, undistorted2.data) ;
        cv::flip(MatUndist2,MatUndist2,1);
        cv::Mat depim2=MatUndist2;
        cv::Mat irim2 = cv::Mat(ir2->height, ir2->width, CV_32FC1, ir2->data);
        cv::flip(irim2,irim2,1);
        cv::Mat MatReg2 = cv::Mat(registered2.height, registered2.width, CV_8UC4, registered2.data);
        cv::flip(MatReg2,MatReg2,1);

        //show images
        cv::resize(cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data), rgb_1, cv::Size(), 0.75, 0.75, cv::INTER_LINEAR );
        cv::flip(rgb_1,rgb_1,1);
        //cv::imshow("rgb", rgb_1);
        //cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data));
        //cv::imshow("ir",  irim/ 65535.0f);
        //cv::moveWindow("ir",0,0);
        //cv::imshow("depth", depim / 8000.0f);
        //cv::imshow("registered",MatReg);
        //cv::moveWindow("registered",0,435);

        cv::resize(cv::Mat(rgb1->height, rgb1->width, CV_8UC4, rgb1->data), rgb1_1, cv::Size(), 0.33, 0.33, cv::INTER_LINEAR );
        cv::flip(rgb1_1,rgb1_1,1);
        //cv::imshow("rgb1", rgb1_1);
        //cv::imshow("rgb1", cv::Mat(rgb2->height, rgb2->width, CV_8UC4, rgb->data));
        //cv::imshow("ir1", irim1 / 65535.0f);
        //cv::moveWindow("ir1",550,0);
        //cv::imshow("depth1", depim1 / 4500.0f);
        //cv::imshow("registered1",MatReg1);
        //cv::moveWindow("registered1",555,435);

        cv::resize(cv::Mat(rgb2->height, rgb2->width, CV_8UC4, rgb2->data), rgb2_1, cv::Size(), 0.33, 0.33, cv::INTER_LINEAR );
        cv::flip(rgb2_1,rgb2_1,1);
        //cv::imshow("rgb2", rgb2_1);
        //cv::imshow("rgb2", cv::Mat(rgb2->height, rgb2->width, CV_8UC4, rgb->data));
        //cv::imshow("ir2", irim2 / 65535.0f);
        //cv::moveWindow("ir2",530*2,0);
        //cv::imshow("depth2", depim2 / 4500.0f);
        //cv::imshow("registered2",MatReg2);
        //cv::moveWindow("registered2",530*2,435);

        cv::Mat mir=irim;///65535.0f;
        //cv::Mat m2(&(irim/65535.0f),true);
        mir.convertTo(mir, CV_8U, 0.00390625f);
        cv::Mat destIR;
        cv::cvtColor(mir, destIR, CV_GRAY2BGR,3);
        destIR.convertTo(destIR, -1, 3, 0);
        //std::cout << "test2:" << destIR.channels() << std::endl ;
        //cv::imshow("test1",destIR);

        cv::Mat mir1=irim1;///65535.0f;
        //cv::Mat m2(&(irim/65535.0f),true);
        mir1.convertTo(mir1, CV_8U, 0.00390625f);
        cv::Mat destIR1;
        cv::cvtColor(mir1, destIR1, CV_GRAY2BGR,3);
        destIR1.convertTo(destIR1, -1, 3, 0);
        //std::cout << "test2:" << destIR1.channels() << std::endl ;
        //cv::imshow("test2",destIR1);

        cv::Mat mir2=irim2;///65535.0f;
        //cv::Mat m2(&(irim/65535.0f),true);
        mir2.convertTo(mir2, CV_8U, 0.00390625f);
        cv::Mat destIR2;
        cv::cvtColor(mir2, destIR2, CV_GRAY2BGR,3);
        destIR2.convertTo(destIR2, -1, 3, 0);
        //std::cout << "test2:" << destIR2.channels() << std::endl ;
        //cv::imshow("test2",destIR2);

        // Create body MAsk
        pMOG2_1->apply(destIR, fgMaskMOG2_1,0.0001f);
        pMOG2_2->apply(destIR1, fgMaskMOG2_2,0.0001f);
        pMOG2_3->apply(destIR2, fgMaskMOG2_3,0.0001f);
        //
        int erosion_type;
        int erosion_elem = 2;
        int erosion_size = 9;///10;
        if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
        else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
        else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

        cv::Mat element = cv::getStructuringElement( erosion_type,
                                                     cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                     cv::Point( erosion_size, erosion_size ) );

        // apply opening operation
        /*cv::erode(fgMaskMOG2_1, fgMaskMOG2_1, element );
        cv::dilate( fgMaskMOG2_1, fgMaskMOG2_1, element );
        cv::erode(fgMaskMOG2_2, fgMaskMOG2_2, element );
        cv::dilate( fgMaskMOG2_2, fgMaskMOG2_2, element );*/
        /// Apply the closing operation
        cv::dilate(fgMaskMOG2_1, fgMaskMOG2_1, element );
        cv::erode( fgMaskMOG2_1, fgMaskMOG2_1, element );

        cv::dilate(fgMaskMOG2_2, fgMaskMOG2_2, element );
        cv::erode( fgMaskMOG2_2, fgMaskMOG2_2, element );

        cv::dilate(fgMaskMOG2_3, fgMaskMOG2_3, element );
        cv::erode( fgMaskMOG2_3, fgMaskMOG2_3, element );

        /*cv::imshow("fgMAskMOG2_1",fgMaskMOG2_1);
        cv::moveWindow("fgMAskMOG2_1",0,0);
        cv::imshow("fgMAskMOG2_2",fgMaskMOG2_2);
        cv::moveWindow("fgMAskMOG2_2",600,0);*/

        // Depth Filtering
        //cv::GaussianBlur(depim,depim,cv::Size(3,3),0,0);//MedianBlur()
        //cv::GaussianBlur(depim1,depim1,cv::Size(3,3),0,0);

        //Segmenting Body from depth image
        cv::Mat d1,d2,d3;
        cv::bitwise_and(depim,depim,d1,fgMaskMOG2_1/255);
        cv::bitwise_and(depim1,depim1,d2,fgMaskMOG2_2/255);
        cv::bitwise_and(depim2,depim2,d3,fgMaskMOG2_3/255);

        // Foreground Segmentation from unnecessary background patches
        func_kinect kf;
        kf.foreground_segment(d1);
        kf.foreground_segment(d2);

        kf.foreground_segment(d3);

        cv::imshow("D1",d1/ 8000.0f);
        cv::moveWindow("D1",0,0);
        cv::imshow("D2",d2/ 8000.0f);
        cv::moveWindow("D2",550,0);
        cv::imshow("D3",d3/ 8000.0f);
        cv::moveWindow("D3",530*2,0);


        // std::cerr<<"hi1";

//Point cloud transformation
        //PCld from Sensor 1 2,3
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud <pcl::PointXYZ>);

        int validNum = kf.convertToXYZPointCloud(d1,MatUndist.cols,MatUndist.rows,fx,fy,cx,cy,cloud);
        int validNum1 = kf.convertToXYZPointCloud(d2,MatUndist.cols,MatUndist.rows,fx1,fy1,cx1,cy1,cloud1);
        int validNum2 = kf.convertToXYZPointCloud(d3,MatUndist.cols,MatUndist.rows,fx2,fy2,cx2,cy2,cloud2);


// 3D Pre-processing in each of 3 point clouds

        if (validNum >10 and validNum1 >10  and validNum2>10 ) {


            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cld(new pcl::PointCloud<pcl::PointXYZ>);
            new_cld = kf.PtCloudPassFilter(cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cld1(new pcl::PointCloud<pcl::PointXYZ>);
            new_cld1 = kf.PtCloudPassFilter(cloud1);

            pcl::PointCloud<pcl::PointXYZ>::Ptr new_cld2(new pcl::PointCloud<pcl::PointXYZ>);
            new_cld2 = kf.PtCloudPassFilter(cloud2);

            //convert to range data Pt_2D
            std::vector<cv::Vec3f> Pt_3D(cloud->points.size(), cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
            std::vector<cv::Vec2i> Pt_2D(cloud->points.size(), cv::Vec2i::all(std::numeric_limits<float>::quiet_NaN()));
            for (int i = 0; i < cloud->points.size(); ++i) {
                pcl::PointXYZ P = cloud->points[i];
                Pt_3D[i] = cv::Vec3f(P.x, P.y, P.z);
                Pt_2D[i] = cv::Vec2i(static_cast<int>(round(((P.x * fx) / P.z) + cx)),
                                     static_cast<int>(round(((P.y * fy) / P.z) + cy)));

            }

            //MEsh generation begins

            if (!new_cld->isOrganized() | !new_cld1->isOrganized() | !new_cld2->isOrganized()) {
                std::cout << "not working" << std::endl;
                protonect_shutdown = true; // shutdown on escape
                //viewerr->close();
                viewer_p1->close();
            }


            pcl::PolygonMesh tri_cld;
            tri_cld = kf.SimpleMeshGen(new_cld);
            pcl::io::savePLYFile("mesh1.ply", tri_cld);
            pcl::fromPCLPointCloud2(tri_cld.cloud, *new_cld);
            new_cld = kf.FillNaN(new_cld);
            //pcl::toPCLPointCloud2( *new_cld,tri_cld.cloud);


            pcl::PolygonMesh tri_cld1;
            tri_cld1 = kf.SimpleMeshGen(new_cld1);
            //pcl::io::savePLYFile ("mesh2.ply", *tri_cld1);
            pcl::fromPCLPointCloud2(tri_cld1.cloud, *new_cld1);
            new_cld1 = kf.FillNaN(new_cld1);
            //pcl::toPCLPointCloud2( *new_cld1,tri_cld1.cloud);

            pcl::PolygonMesh tri_cld2;
            tri_cld2 = kf.SimpleMeshGen(new_cld2);
            //pcl::io::savePLYFile ("mesh3.ply", *tri_cld2);
            pcl::fromPCLPointCloud2(tri_cld2.cloud, *new_cld2);
            new_cld2 = kf.FillNaN(new_cld2);
            //pcl::toPCLPointCloud2( *new_cld2,tri_cld2.cloud);



            //d1= kf.convertXYZPointCloud2DepthImage(new_cld);
            //cv::imshow("D1",d1/ 8.0f);
            // Boundary points
            std::vector<pcl::PointIndices> label_indices, label_indices1, label_indices2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr
                    occluding_edges(new pcl::PointCloud<pcl::PointXYZ>),
                    occluded_edges(new pcl::PointCloud<pcl::PointXYZ>),
                    boundary_edges(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr
                    occluding_edges1(new pcl::PointCloud<pcl::PointXYZ>),
                    occluded_edges1(new pcl::PointCloud<pcl::PointXYZ>),
                    boundary_edges1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr
                    occluding_edges2(new pcl::PointCloud<pcl::PointXYZ>),
                    occluded_edges2(new pcl::PointCloud<pcl::PointXYZ>),
                    boundary_edges2(new pcl::PointCloud<pcl::PointXYZ>);

            kf.BoundaryEdges(new_cld, label_indices, *occluded_edges, *occluding_edges, *boundary_edges);
            kf.BoundaryEdges(new_cld1, label_indices1, *occluded_edges1, *occluding_edges1, *boundary_edges1);
            kf.BoundaryEdges(new_cld2, label_indices2, *occluded_edges2, *occluding_edges2, *boundary_edges2);


            //MEsh genration ends


            //std::cout << cv::Mat(Pt_2D) <<std::endl;


            // cv::Mat_<float> focal_matrix = cv::Mat::eye(3, 3, CV_32FC1);

            // focal_matrix(0, 0) = fx;  focal_matrix(1, 1) = fx;
            // Translation vector ///
            //cv::Vec3f optics_trans(cx,cy,0.0);





            // Fusion of data
            //Transformation
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_t12(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_t13(new pcl::PointCloud<pcl::PointXYZ>);
            /*cloud1_t13->height = depthHeight;
            cloud1_t13->width = depthWidth;
            cloud1_t13->is_dense = false;
            cloud1_t13->points.resize (cloud1_t13->height * cloud1_t13->width);*/


            //cloud1->points.resize (cloud1->height * cloud1->width);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Matrix4f T_12 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f T_13 = Eigen::Matrix4f::Identity();

            T_12(0, 0) = -0.385985601341530f;
            T_12(0, 1) = 0.366999522576863f;
            T_12(0, 2) = -0.846360718597793f;
            T_12(1, 0) = -0.325533910406865f;
            T_12(1, 1) = 0.804250742010366f;
            T_12(1, 2) = 0.497200580400900f;
            T_12(2, 0) = 0.863158611572771f;
            T_12(2, 1) = 0.467431379353303f;
            T_12(2, 2) = -0.190958416582344f;
            T_12(0, 3) = 1.74351720495226f;
            T_12(1, 3) = -0.949919471763809f;
            T_12(2, 3) = 2.72405301758183f;


            T_13(0, 0) = -0.582693017114721f;
            T_13(0, 1) = -0.227322110860192f;
            T_13(0, 2) = 0.780252206481860f;
            T_13(1, 0) = 0.302256868897643f;
            T_13(1, 1) = 0.830592679137605f;
            T_13(1, 2) = 0.467714214630270f;
            T_13(2, 0) = -0.754393553133865f;
            T_13(2, 1) = 0.508370395752038f;
            T_13(2, 2) = -0.415270884740285f;
            T_13(0, 3) = -1.56137244935603f;
            T_13(1, 3) = -0.990840157731375f;
            T_13(2, 3) = 2.90650203016432f;

            //std::cout << T_12 << std::endl;
            //std::cout << T_23 << std::endl;


            //pcl::transformPointCloud (*new_cld1, *cloud1_t12, T_12);
            //pcl::transformPointCloud (*new_cld2, *cloud1_t13, T_13);


            pcl::fromPCLPointCloud2(tri_cld1.cloud, *cloud1_t12);
            pcl::transformPointCloud(*cloud1_t12, *cloud1_t12, T_12);
            pcl::toPCLPointCloud2(*cloud1_t12, tri_cld1.cloud);

            //pcl::io::savePLYFile ("mesh2.ply", tri_cld1);


            pcl::fromPCLPointCloud2(tri_cld2.cloud, *cloud1_t13);
            pcl::transformPointCloud(*cloud1_t13, *cloud1_t13, T_13);
            pcl::toPCLPointCloud2(*cloud1_t13, tri_cld2.cloud);

            // pcl::io::savePLYFile ("mesh3.ply", tri_cld2);

            pcl::fromPCLPointCloud2(tri_cld.cloud, *cloud_fused);

            pcl::PolygonMesh fused_tri;


            fused_tri = tri_cld;
            int update_index = fused_tri.cloud.height * fused_tri.cloud.width;
            for (size_t i = 0; i < tri_cld1.polygons.size(); i++) {
                pcl::Vertices face = tri_cld1.polygons[i];
                face.vertices[0] += update_index;
                face.vertices[1] += update_index;
                face.vertices[2] += update_index;
                fused_tri.polygons.push_back(face);
            }
            *cloud_fused = *cloud_fused + *cloud1_t12;

            pcl::toPCLPointCloud2(*cloud_fused, fused_tri.cloud);


            int update_index1 = fused_tri.cloud.height * fused_tri.cloud.width;
            for (size_t i = 0; i < tri_cld2.polygons.size(); i++) {
                pcl::Vertices face = tri_cld2.polygons[i];
                face.vertices[0] += update_index1;
                face.vertices[1] += update_index1;
                face.vertices[2] += update_index1;
                fused_tri.polygons.push_back(face);
            }

            *cloud_fused = *cloud_fused + *cloud1_t13;

            //cloud_fused->sensor_orientation_ = cloud->sensor_orientation_;

            //cloud_fused->sensor_origin_ = cloud->sensor_origin_;



            std::vector<cv::Vec2i> intra_connectivity;
            std::vector<float> intra_weights;

            kf.IntraEdgeConnectivity(fused_tri, cloud_fused, intra_connectivity, intra_weights);

            pcl::toPCLPointCloud2(*cloud_fused, fused_tri.cloud);

            //pcl::io::savePLYFile ("mesh4.ply", fused_tri);

            // transformed boundary points

            pcl::transformPointCloud(*boundary_edges1, *boundary_edges1, T_12);
            pcl::transformPointCloud(*boundary_edges2, *boundary_edges2, T_13);
            std::vector<int> boundary_indices = label_indices[0].indices;
            std::vector<int> boundary_indices1 = label_indices1[0].indices;
            std::vector<int> boundary_indices2 = label_indices2[0].indices;
            for (size_t i = 0; i < boundary_indices1.size(); i++) {
                boundary_indices1[i] += update_index;
            }
            for (size_t i = 0; i < boundary_indices2.size(); i++) {
                boundary_indices2[i] += update_index1;
            }

            //Interconnectivity edges
            std::vector<cv::Vec2i> inter_connectivity;
            std::vector<float> inter_weights;
            kf.InterEdgeConnectivity(new_cld, boundary_edges1, boundary_indices1, 0, inter_connectivity, inter_weights);
            //std::cout<< "inter_ size1: "<< inter_weights.size()<<std::endl;
            kf.InterEdgeConnectivity(cloud1_t12, boundary_edges2, boundary_indices2, update_index, inter_connectivity,
                                     inter_weights);
            //std::cout<< "inter_ size2: "<< inter_weights.size()<<std::endl;
            kf.InterEdgeConnectivity(cloud1_t13, boundary_edges, boundary_indices, update_index1, inter_connectivity,
                                     inter_weights);
            //std::cout<< "inter_ size3: "<< inter_weights.size()<<std::endl;

            // Graph building;

            Graph G1(cloud_fused->points.size()), G2(cloud_fused->points.size());

            kf.GraphBuilding(intra_connectivity, intra_weights, inter_connectivity, inter_weights, G1);
            G2 = G1;
            const size_t E = boost::num_edges(G1), V = boost::num_vertices(G1);
            std::cout << "Vertices No" << V << std::endl;
            std::cout << "Edge No" << E << std::endl;
            property_map<Graph, edge_weight_t>::type weightmap;
            // finding Centroid point to be used as starting vertex
            unsigned long u_vert, cent_vert; //starting vertex
            unsigned long periferal_node = 0;
            std::vector<unsigned long> secondary_node;
            vertex_descriptor s;
            std::vector<vertex_descriptor> p(V);
            std::vector<float> geodist(V);
            std::vector<unsigned long> extrema;
            extrema.clear();
            //Centroid Geodesic
            kf.FillNaN(cloud_fused);
            Eigen::Vector4f centroid;
            kf.FindNearestPt2CentroidPCL(cloud_fused, centroid, cent_vert);
            u_vert = cent_vert;

            float highest_geo;
            weightmap = get(boost::edge_weight, G1);
            for (int et = 0; et < K_EX; ++et) {
                if (et > 0) u_vert = periferal_node;

                s = vertex(u_vert, G1);
                dijkstra_shortest_paths(G1, s,
                                        predecessor_map(boost::make_iterator_property_map(p.begin(), boost::get(
                                                boost::vertex_index, G1))).
                                                distance_map(boost::make_iterator_property_map(geodist.begin(),
                                                                                               boost::get(
                                                                                                       boost::vertex_index,
                                                                                                       G1))).
                                                weight_map(weightmap));

                //std::vector<float>::iterator result1;

                //result1 = std::max_element(geodist.begin(), geodist.end());

                //float val= geodist[s];
                //periferal_node=0;
                highest_geo = 0.0f;
                for (size_t i = 0; i < V; ++i) {
                    if (geodist[i] == std::numeric_limits<float>::max() | geodist[i] == 0.0f) {
                        geodist[i] = std::numeric_limits<float>::quiet_NaN();
                        continue;
                    }
                    if (geodist[i] > highest_geo) {
                        highest_geo = geodist[i];
                        periferal_node = i;
                    }
                }
                //highest_geo= *result1;
                //std::cout << "test1 : " << highest_geo <<std::endl;

                //periferal_node= static_cast<unsigned long>(std::distance(geodist.begin(),result1));
                extrema.push_back(periferal_node);
                // std::cout << "test2 : " << periferal_node <<std::endl;

                boost::add_edge(u_vert, periferal_node, Weight(0.0f), G1);
                boost::add_edge(cent_vert, periferal_node, Weight(0.0f), G1);
                //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fused_I(new pcl::PointCloud<pcl::PointXYZI>);

                //kf.ADDGeoIntensity(cloud_fused, geodist, cloud_fused_I) ;
                //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_fused_I,"intensity");

                /*if (!(viewer_p1->updatePointCloud<PointXYZI>(cloud_fused_I,intensity_distribution,"Cloud with intensity")))
                 viewer_p1->addPointCloud<PointXYZI>(cloud_fused_I,intensity_distribution,"Cloud with intensity"); */
                //viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud with intensity");


            }

            //std::cout << "extrema points : " << extrema[0] <<" " << extrema[1] << std::endl;


            // Point visualization

            viewer_p1->setBackgroundColor(0, 0, 0);
            //fused cloud
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_fused, 20, 20, 230);
            if (!(viewer_p1->updatePointCloud<pcl::PointXYZ>(cloud_fused, cloud_color, "cloud")))
                viewer_p1->addPointCloud<pcl::PointXYZ>(cloud_fused, cloud_color, "cloud");
            viewer_p1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
            //Rendering pt  from sensor 1


            /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (new_cld, 20, 20, 230);
            if(!(viewer_p1->updatePointCloud<pcl::PointXYZ> (new_cld,cloud_color, "cloud")))
                viewer_p1->addPointCloud<pcl::PointXYZ> (new_cld,cloud_color, "cloud");
            viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  */



            /* pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_t12_color (boundary_edges, 20, 150, 20);
             if(!(viewer_p1->updatePointCloud<pcl::PointXYZ> (boundary_edges,cloud1_t12_color, "cloud1")))
                 viewer_p1->addPointCloud<pcl::PointXYZ> (boundary_edges,cloud1_t12_color, "cloud1");
             viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1"); */

            /*
            if (!(viewer_p1->updateSphere(cloud_fused->points[cent_vert], 0.05, 0.5, 0.5, 0.0, "sphere")))
                viewer_p1->addSphere(cloud_fused->points[cent_vert], 0.05, 0.5, 0.5, 0.0, "sphere", 0);

            pcl::PointXYZ searchPoint;

            searchPoint.x = centroid[0];
            searchPoint.y = centroid[1];
            searchPoint.z = centroid[2];
            std::cout << searchPoint << std::endl;
            /*      if (! (viewer_p1->updateSphere(searchPoint, 0.05, 0.5, 0.5, 0.0, "sphere")))
                      viewer_p1->addSphere (searchPoint, 0.05, 0.5, 0.5, 0.0, "sphere");   */


            for (int et = 0; et < K_EX; ++et) {
                if (!(viewer_p1->updateSphere(cloud_fused->points[extrema[et]], 0.08, 0.5, 0.0, 0.5, extremaStr[et])))
                    viewer_p1->addSphere(cloud_fused->points[extrema[et]], 0.08, 0.5, 0.0, 0.5, extremaStr[et]);

            }

            /*if (! (viewer_p1->updateSphere(cloud_fused->points[periferal_node], 0.08, 0.5, 0.0, 0.5, extremaStr[0])))
                viewer_p1->addSphere (cloud_fused->points[periferal_node], 0.08, 0.5, 0.0, 0.5, extremaStr[0],0); */



            //Rendering ptcloud from sensor 2
            /*     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_t12_color (cloud1_t12, 20, 150, 20);
                 viewer_p1->updatePointCloud<pcl::PointXYZ> (cloud1_t12,cloud1_t12_color, "cloud1");
                     viewer_p1->addPointCloud<pcl::PointXYZ> (cloud1_t12,cloud1_t12_color, "cloud1");
                     viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");*/

            //Rendering ptcloud from sensor 3
            /*          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_t13_color (cloud1_t13, 150, 20, 20);
                      viewer_p1->updatePointCloud<pcl::PointXYZ> (cloud1_t13,cloud1_t13_color, "cloud2");
                          viewer_p1->addPointCloud<pcl::PointXYZ> (cloud1_t13,cloud1_t13_color, "cloud2");
                          viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");*/


            viewer_p1->addCoordinateSystem(1.0);
            //viewer_p1->setCameraPosition(0,0,0,-1,-1,0,0,0,0);
            viewer_p1->spinOnce();
            //viewer_p1->saveScreenshot("Image.png");
        }
        ////////////

      ///////

  /////////////////////////////////
        if (count % 10 == 0 and record == true)
        {
            data_count=count/10;

            std::cout << data_count << "\n";


            std::string filename;
            std::string  index2 = static_cast<std::ostringstream*>( &(std::ostringstream() << data_count) )->str();

// Sensor 0
            std::string sensor="cam0/";//id of sensor
            std::string foldername1=foldername+sensor;

            //Color
            std::string app = ".jpg";
            filename = index2 + app;
            //std::cout << filename << "\n";
            std::string type="rgb/";
            std::string foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),rgb_1);
            // IR
            type="ir/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            //std::cout << foldername11 + filename << "\n";
            for (int i = 0; i < (ir->width*ir->height); i++)
            {
                int indexi = i / ir->width; //the height
                int indexj = i % ir->width; //the width

                int a = irim.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();
            //Depth
            type="depth/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth->width*depth->height); i++)
            {
                int indexi = i / depth->width; //the height
                int indexj = i % depth->width; //the width

                int a = depim.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();
            //Registered image
            type="registered/";
            app=".jpg";
            filename=index2+app;
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),MatReg);

            //Depth_masked
            type="depth_mask/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth->width*depth->height); i++)
            {
                int indexi = i / depth->width; //the height
                int indexj = i % depth->width; //the width

                int a = d1.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();
            //Mask
            app= ".png";
            filename = index2+app;
            type="mask/";
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),fgMaskMOG2_1);
// Sensor 1
            sensor="cam1/";//id of sensor
            foldername1=foldername+sensor;

            //Color
            app = ".jpg";
            filename = index2 + app;
            //std::cout << filename << "\n";
            type="rgb/";
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),rgb1_1);
            // IR
            type="ir/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            //std::cout << foldername11 + filename << "\n";
            for (int i = 0; i < (ir1->width*ir1->height); i++)
            {
                int indexi = i / ir1->width; //the height
                int indexj = i % ir1->width; //the width

                int a = irim1.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();
            //Depth
            type="depth/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth1->width*depth1->height); i++)
            {
                int indexi = i / depth1->width; //the height
                int indexj = i % depth1->width; //the width

                int a = depim1.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();

            //Registered image
            type="registered/";
            app=".jpg";
            filename=index2+app;
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),MatReg1);
            //Depth masked
            type="depth_mask/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth->width*depth->height); i++)
            {
                int indexi = i / depth->width; //the height
                int indexj = i % depth->width; //the width

                int a = d2.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();

            //Mask
            app= ".png";
            filename = index2+app;
            type="mask/";
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),fgMaskMOG2_2);

// Sensor 2
            sensor="cam2/";//id of sensor
            foldername1=foldername+sensor;

            //Color
            app = ".jpg";
            filename = index2 + app;
            //std::cout << filename << "\n";
            type="rgb/";
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),rgb2_1);
            // IR
            type="ir/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            //std::cout << foldername11 + filename << "\n";
            for (int i = 0; i < (ir2->width*ir2->height); i++)
            {
                int indexi = i / ir2->width; //the height
                int indexj = i % ir2->width; //the width

                int a = irim2.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();
            //Depth
            type="depth/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth2->width*depth2->height); i++)
            {
                int indexi = i / depth2->width; //the height
                int indexj = i % depth2->width; //the width

                int a = depim2.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();

            //Registered image
            type="registered/";
            app=".jpg";
            filename=index2+app;
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),MatReg2);

            //Depth masked
            type="depth_mask/";
            app = ".txt";
            filename = index2 + app;
            foldername11=foldername1+type;
            //std::cout << foldername11 << "\n";
            myfile.open((foldername11 + filename).c_str(),std::ofstream::out);
            for (int i = 0; i < (depth2->width*depth2->height); i++)
            {
                int indexi = i / depth2->width; //the height
                int indexj = i % depth2->width; //the width

                int a = d3.at<float>(indexi, indexj);
                myfile << a <<" ";
                if (indexj == 0)
                {
                    myfile << "\n";
                }

            }
            myfile.close();

            //Mask
            app= ".png";
            filename = index2+app;
            type="mask/";
            foldername11=foldername1+type;
            cv::imwrite((foldername11+filename),fgMaskMOG2_3);

        }
/*
    cv::imshow("undistorted", cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data) / 4500.0f);
    cv::imshow("registered", cv::Mat(registered.height, registered.width, CV_8UC4, registered.data));

    cv::imshow("undistorted", cv::Mat(undistorted1.height, undistorted1.width, CV_32FC1, undistorted1.data) / 4500.0f);
    cv::imshow("registered", cv::Mat(registered1.height, registered1.width, CV_8UC4, registered1.data));
*/
        int key = cv::waitKey(1);
        if (key > 0 && ((key & 0xFF) == 27)){
            protonect_shutdown = true; // shutdown on escape
            //viewerr->close();
            viewer_p1->close();
        }
        listener.release(frames);
        listener1.release(frames1);
        listener2.release(frames2);
        //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
    }

    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();
    dev1->stop();
    dev1->close();
    dev2->stop();
    dev2->close();

    delete registration;
    delete registration1;
    delete registration2;

    return 0;
}
