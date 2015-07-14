//
//  videoprocessing.h
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/8/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "opencv/cv.h"
#include "opencv/cxmisc.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include "opencv/cxmisc.h"
#include <iostream>
#include <math.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

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