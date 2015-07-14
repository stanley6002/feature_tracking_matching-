//
//  main.cpp
//  Feature_tracking_Project
//
//  Created by C-HChang on 7/11/15.
//  Copyright (c) 2015 C-HChang. All rights reserved.
//

#include <iostream>



#include "videoprocessing.h"
#include "matchingandtracking.h"

using namespace std;
using namespace cv;

CvCapture *camCapture;

int Img_width;
int Img_height;

IplImage* skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            return NULL;
        }
    }

    return cvQueryFrame(capture);
}
IplImage* plot_two_imagesf(IplImage *IGray, IplImage *IGray1,CvPoint*r_pt, CvPoint* l_pt, int numpts)
{
    CvPoint newmatched;
    CvPoint matched;
    int  ttw  =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++)
    {
        for (int j=0; j<ttw; j++)
        {
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }

    CvPoint pt1,pt2;
    pt1.x=0;
    pt1.y=0;
    pt2.x= (IGray->width)*2;
    pt2.y= (IGray->height);



    //if(skipFrame)
    //    cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(256,0,0), 4, 8, 0 );
    //else
        cvRectangle(Imagedisplay,pt1 , pt2, CV_RGB(0,256,0), 4, 8, 0 );
    for (int i=0;i<numpts; i++)
    {

        //newmatched.x= (int)(l_pt[i].x)+(IGray->width)+(0.5*(IGray->width));
        //newmatched.y= (int)(l_pt[i].y)+(0.5*(IGray->height));
        //matched.x = (int) r_pt[i].x+(0.5*(IGray->width));
        //matched.y = (int) r_pt[i].y+(0.5*(IGray->height));

        newmatched.x= (int)(l_pt[i].x)+(IGray->width);
        newmatched.y= (int)(l_pt[i].y);
        matched.x = (int) r_pt[i].x;
        matched.y = (int) r_pt[i].y;

        cvLine(Imagedisplay,
               cvPoint( matched.x, matched.y ),
               cvPoint( newmatched.x, newmatched.y ),
               CV_RGB(256,256,256)
               );


        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(0,255,0),2,6,0);
        cvCircle(Imagedisplay, matched, 3, CV_RGB(0,255,0), 2,6,0);

    }  
    return(Imagedisplay);
}


int main(int argc, const char * argv[]) {

    camCapture = cvCaptureFromCAM(CV_CAP_ANY);

    Img_width=320;
    Img_height=240;


    if (!(camCapture))
    {
        cout << "Failed to capture from camera" << endl;
        return 0;
    }

    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_WIDTH,  Img_width);
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_HEIGHT, Img_height);

    cout << "Camera opened successfully" << endl;

    IplImage *cameraFrame;

    bool _1stframe=true;
    //bool _1sttrack=true;

    cameraFrame = cvCreateImage(cvSize (Img_width,Img_height), IPL_DEPTH_8U,3);

    IplImage * imgA=0;
    IplImage * imgB=0;
    IplImage * imgC=0;

    IplImage * imgGrayA=0;
    IplImage * imgGrayB=0;

    //Img_width=  320 ;
    //Img_height= 240 ;

    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);

    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);

    //IplImage* Two_image = 0;

    vector<Point2f> tempCorners;

    IplImage* frame;

    bool readfromvideo= 1 ;
    VideoProcessing VideoProcessing (Img_width, Img_height, readfromvideo);

    //FeaturePts FeaturePts;

    //FeaMapPoints FeaMapPoints;

    /// flow control parameters

    int firstcapture = 1;
    bool SkipthisFrame=0;
    bool ThirdFrame=0;
    int loop =0;
    //  flow control
    do
        if (camCapture)
        {
            int fps = ( int )cvGetCaptureProperty( camCapture, CV_CAP_PROP_FPS );
            if ( !camCapture )
            {
                fprintf( stderr, "Cannot open AVI!\n" );
                return 1;
            }
            // exit if it reaches the last frame
            if( ! cvGrabFrame(camCapture))
            {
                break;
            }

            if ( _1stframe)
            {
                frame = cvQueryFrame(camCapture);
                frame = skipNFrames(camCapture, 10);
                imgB  = cvCloneImage(frame);
            }

            if( ! _1stframe)
            {
                bool CaptureFrames=0;
                if (! SkipthisFrame)
                {
                    frame = skipNFrames(camCapture, 0);
                    frame = VideoProcessing.CaptureFrame(camCapture);

                    if (! CaptureFrames)
                    {
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames =1;
                        }
                    }
                }
                else
                {
                    cout<<"Frame skipped "<<endl;
                    frame = VideoProcessing.CaptureFrame(camCapture);

                    if (! CaptureFrames)
                    {
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames =1;
                        }
                    }

                }
                if(CaptureFrames==1)
                {
                    imgA= frame;  // new frame //
                    imgC= cvCloneImage(imgB);  // previous frame //

                    cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);
                    cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);

                    std::vector<CvPoint2D32f> match_query;
                    std::vector<CvPoint2D32f> match_train;

                    //LKFeatures LKFeatures (imgGrayA,imgGrayB, LKFeatures. BRIEF_descriptor);
                    //LKFeatures.FeaturesMatched (match_query, match_train);

                    SIFTfeature SIFTfeature(imgGrayA, imgGrayB,2, 0.05);
                    SIFTfeature.SIFTfeaturematch(match_query, match_train);

                    //SURFfeature  SURFfeature(imgGrayA, imgGrayB,  600 );
                    //SURFfeature. SURFfeaturematch(match_query, match_train);

                    //FAST_ FAST_(2, imgGrayA, imgGrayB, FAST_. SURF_descriptor);
                    //FAST_. FAST_tracking(match_query, match_train);
                    
                    // ORBfeature  ORBfeature(imgGrayA, imgGrayB,0.01,0.01);
                    // ORBfeature. ORBfeaturematch(match_query, match_train);

                    int numpts= (int)match_train.size();

                    CvPoint* r_pt = new CvPoint[numpts];
                    CvPoint* l_pt = new CvPoint[numpts];

                    for(int i=0 ; i< numpts;i++)
                    {
                        r_pt[i].x=match_query[i].x;
                        r_pt[i].y=match_query[i].y;

                        l_pt[i].x=match_train[i].x;
                        l_pt[i].y=match_train[i].y;

                    }

                    IplImage* Two_image=plot_two_imagesf(imgGrayA, imgGrayB, r_pt,  l_pt, numpts);
                    cvShowImage("test", Two_image);
                    cvReleaseImage(&Two_image);
                    cvWaitKey(1);

                    imgB= cvCloneImage(frame);
                    ThirdFrame= true;
                }
            }
            firstcapture=0;
            _1stframe=false;

        }
    while (true) ;
    cvReleaseImage(&frame);
    cvReleaseImage(&imgA);
    cvReleaseImage(&imgB);
    cvReleaseImage(&imgC);
    cvReleaseImage(&imgGrayA);
    cvReleaseImage(&imgGrayB);

}
