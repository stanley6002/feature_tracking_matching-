//
//  videoprocessing.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/8/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include <cv.h>
#include <cxmisc.h>
#include <cxcore.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <iostream>
#include <math.h>
#include <features2d/features2d.hpp>
#include <nonfree/nonfree.hpp>


class VideoProcessing
{
   
public:
   
    int ImgWidth ;
    int ImgHeight;
    
    bool captureNextFrame;
    bool captureThirdFrame;
    
    IplImage* imgGray1;
    IplImage* imgGray2;
    
    IplImage* Image1;
    IplImage* Image2;
    
    IplImage* frame; 
    
    VideoProcessing (int Height , int Width, bool readfromvideo);
    
    IplImage*  CaptureFrame(CvCapture* camCapture);

    bool CaptureNextframe();
    
    ~ VideoProcessing ();
    IplImage* capture(CvCapture* camCapture);
    
    IplImage* GrayImg(IplImage* image);
    
    IplImage* ToGrayImg();

    void skipNFrames(CvCapture* capture, int n);

     IplImage*  CaptureFirstFrame(CvCapture* camCapture);

private:
 
    bool count_frame; 
    bool readvideo;
     
    //bool countFrameRest;
};