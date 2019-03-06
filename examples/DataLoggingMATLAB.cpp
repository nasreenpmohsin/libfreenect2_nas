//
// Created by nmohsin on 03/02/19.
//

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
        /*kf.foreground_segment(d1);
        kf.foreground_segment(d2);

        kf.foreground_segment(d3);*/

        cv::imshow("D1",d1/ 8000.0f);
        cv::moveWindow("D1",0,0);
        cv::imshow("D2",d2/ 8000.0f);
        cv::moveWindow("D2",550,0);
        cv::imshow("D3",d3/ 8000.0f);
        cv::moveWindow("D3",530*2,0);

        ////////////

        ///////

        /////////////////////////////////
        if (count % 5 == 0)
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
