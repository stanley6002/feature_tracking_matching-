//
//  videoprocessing.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/8/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "videoprocessing.h"
using namespace std;
using namespace cv;

IplImage* VideoProcessing :: capture(CvCapture* camCapture)
{
     frame = cvQueryFrame(camCapture);
     Image1=cvCloneImage(frame);
    

    return(Image1);

}

bool VideoProcessing :: CaptureNextframe()
{
    return(captureNextFrame)  ;
}

void VideoProcessing :: skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            cout<<"cannot capture frame";
        }
    }
}
VideoProcessing:: VideoProcessing (int Width , int Height, bool readfromvideo)
{
     ImgWidth = Width;
     ImgHeight = Height;
     count_frame= FALSE;
     readvideo = readfromvideo;

     imgGray1  = cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 1);
     imgGray2  = cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 1);
    
     Image1=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     Image2=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);

     frame = cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     //Image3=  cvCreateImage(cv::Size(Width, Height), IPL_DEPTH_8U, 3);
     //frame=  cvCreateImage(cv::Size(ImgWidth, ImgHeight), IPL_DEPTH_8U, 3);;
     
     captureNextFrame = FALSE;
     captureThirdFrame= FALSE;
     
}

IplImage* VideoProcessing :: CaptureFrame(CvCapture* camCapture)
{
vector<cv::Point2f> cornerfirst, cornersecond;

    if (count_frame)
    {   

        if (readvideo)
        {
            IplImage* tempframe = cvQueryFrame(camCapture);
            cvResize(tempframe, frame);
            Image2= cvCloneImage(frame);
        }

        //Mat MatGray1, MatGray2;
        else
        {
            frame = cvQueryFrame(camCapture);
            Image2= cvCloneImage(frame);
        }
        
        cvCvtColor(Image1,imgGray1, CV_BGR2GRAY);  
        cvCvtColor(Image2,imgGray2, CV_BGR2GRAY);
        vector<cv::Point2f> corners;

        Mat MatGray1 (imgGray1 );
        Mat MatGray2 (imgGray2 );


        goodFeaturesToTrack(MatGray1, cornerfirst,  400, 0.001, 16);
        goodFeaturesToTrack(MatGray2, cornersecond, 400, 0.001, 16);

        MatGray1.release();
        MatGray2.release();
        
        //cout<< cornerfirst.size()<<" "<<cornersecond.size()<<endl;
    
    if ((int) cornerfirst.size() >= cornersecond.size())
    { 
        frame = Image1;
        cout<<"captured first frame"<<endl;
   
    }
    else
    {
        frame = Image2;
        cout<<"captured second frame"<<endl;
    }
    
        count_frame= FALSE;
        captureNextFrame=true;
        
    }

    if( ! count_frame)
    {
        if (readvideo)
        {
            IplImage* tempframe = cvQueryFrame(camCapture);
            cvResize(tempframe, frame);
            Image1= cvCloneImage(frame);
            count_frame = true;
        }

        //Mat MatGray1, MatGray2;
        else
        {
            VideoProcessing::frame = cvQueryFrame(camCapture);
            VideoProcessing::Image1= cvCloneImage(frame);
            count_frame = true;
            
        }
    }

    //count_frame++;
    //countFrameReset= true ;
    
    return (frame);
}

IplImage*  VideoProcessing::CaptureFirstFrame(CvCapture* camCapture)
{

    if (readvideo)
    {
        IplImage* tempframe = cvQueryFrame(camCapture);
        cvResize(tempframe, frame);
        Image2= cvCloneImage(frame);
    }

    //Mat MatGray1, MatGray2;
    else
    {
        frame = cvQueryFrame(camCapture);
        Image2= cvCloneImage(frame);
    }
    return(frame);
}

VideoProcessing::~VideoProcessing()
{
    cvReleaseImage(&Image1);
    cvReleaseImage(&Image2);
    cvReleaseImage(&imgGray1);
    cvReleaseImage(&imgGray2);
    cvReleaseImage(&frame);
}