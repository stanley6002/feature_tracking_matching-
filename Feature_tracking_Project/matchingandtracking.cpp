//
//  matchingandtracking.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/9/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "matchingandtracking.h"


using namespace std;
using namespace cv;

 FAST_ :: FAST_(int level, IplImage* imgGrayA, IplImage* imgGrayB, Function Descriptor)
{
    
    FAST_::Level= level;    

    ImageGray1 = cv::cvarrToMat(imgGrayA);
    
    ImageGray2 = cv::cvarrToMat(imgGrayB);
    
    if (Descriptor == SURF_descriptor)
    { 
        Surf_activate=1;
    }
    else 
    {
        Surf_activate=0;
    
    }
}

void FAST_ :: FAST_tracking(std::vector<CvPoint2D32f>& match_query, std::vector<CvPoint2D32f>& match_train)
 {


     if(! Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level, TRUE);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level, TRUE);


         FAST_descriptor = new cv::BriefDescriptorExtractor(64);

         FAST_descriptor->compute(ImageGray1,  FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2,  FAST_train_kpts,FAST_train_desc);

         FAST_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

         std::vector<cv::KeyPoint> test_kpts;
         FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);

         //warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);
         //cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 100, 100 );
         //FAST_matcher->match(FAST_query_desc, FAST_train_desc, FAST_matche, FAST_mask);
         int i=0;

         for (; i< FAST_matche.size(); i++)
             //for (; i< FAST_matcher.size(); i++)
         {
             int queryIdx = FAST_matche[i].queryIdx;
             int trainIdx = FAST_matche[i].trainIdx;

             //int queryIdx = FAST_matcher[i].queryIdx;
             //int trainIdx = FAST_matcher[i].trainIdx;

             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);

         }
     }

     if(Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level, TRUE);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level, TRUE);



         FAST_descriptor=new cv::OrbDescriptorExtractor(4,2);

         FAST_descriptor->compute(ImageGray1, FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2, FAST_train_kpts, FAST_train_desc);

         std::vector<cv::DMatch> FAST_matcher;

         std::vector<cv::KeyPoint> test_kpts;

         //FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);
         //warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);
         //cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 250, 250);

         //new cv::SiftFeatureDetector();
         //detector = new SiftFeatureDetector;
         //cv::DescriptorExtractor

         cv::BruteForceMatcher<cv::L2<float> > matcher;
         matcher.match(FAST_query_desc, FAST_train_desc, FAST_matcher);

         //std::vector<std::vector<cv::DMatch> > matches;

         int i=0;
         for (; i< FAST_matcher.size(); i++)
         {

             int queryIdx = FAST_matcher[i].queryIdx;
             int trainIdx = FAST_matcher[i].trainIdx;

             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);
             
         }
     }
 }

 FAST_::~ FAST_()
{
    //cvReleaseImage(&ImageGray1);
    //cvReleaseImage(&ImageGray2);
    ImageGray1.release();
    ImageGray2.release();
}

void FAST_ :: warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out)
{
    vector<cv::Point2f> pts;
    keypoints2points(in, pts);
    vector<cv::Point2f> pts_w(pts.size());
    cv::Mat m_pts_w(pts_w);
    
    
    perspectiveTransform(cv::Mat(pts), m_pts_w, H);
    
    
    points2keypoints(pts_w, out);
}

void FAST_ :: keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

void FAST_ :: points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        
        out.push_back(cv::KeyPoint(in[i], 1));
    }
}

LKFeatures :: LKFeatures(IplImage* imgGrayA, IplImage* imgGrayB, LKFeatures::Function input)
{

  int Numberofcorner=400;
  int threshold1=0.01;
  int Windowsize=7;
  int Wsize =16;
  ParametersInitialized(Numberofcorner, threshold1, Wsize, input);

   ImageGray1 = cv::cvarrToMat(imgGrayA);

   ImageGray2 = cv::cvarrToMat(imgGrayB);

  //ImageGray2 = cvCloneImage(imgGrayB);
  LKFeaturesTracking(input);

}
void LKFeatures:: ParametersInitialized (int Numberofcorner, int threshold1, int Wsize, LKFeatures::Function input)
{
      NumberCorn = Numberofcorner;
      Threshold  = threshold1;
      W_size = Wsize;
      Windowsize =7;
    
    if(input == Optical_flow)
    {
        UseOptical_flow= true;
     }
    else
    {
        UseOptical_flow= false;
    }    
           
}

void LKFeatures::  LKFeaturesTracking (Function input)
{
switch(input)
    {

       case Optical_flow:
         {
           
           goodFeaturesToTrack(ImageGray1, corners,  400, 0.01,10);
           cornerSubPix( ImageGray1, corners, cv::Size(9,9) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
           calcOpticalFlowPyrLK(ImageGray1, ImageGray2, corners, nextPts, status, err, cv::Size(40,40));
           break;
         }

    case BRIEF_descriptor:
         {
           goodFeaturesToTrack(ImageGray1,  LK_query_kpts, 400, 0.001, 8);
           cornerSubPix( ImageGray1, LK_query_kpts, cv::Size(9,9) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
           calcOpticalFlowPyrLK(ImageGray1, ImageGray2, LK_query_kpts, LK_train_kpts, status, err, cv::Size(40,40));

           LK_descriptor = new cv::BriefDescriptorExtractor(32);


           int size_= (int) LK_query_kpts.size();


           std::vector<cv::KeyPoint> temp_query;
           //temp_query.resize(size_);

           std::vector<cv::KeyPoint> temp_train;

          //// CONVERT 2D pointf to keypoint
           for(int i=0;i<size_;i++)

           {
              temp_query.push_back(cv::KeyPoint(LK_query_kpts[i], 1.0f));
              temp_train.push_back(cv::KeyPoint(LK_train_kpts[i], 1.0f));
           }

           LK_descriptor->compute(ImageGray1,  temp_query,  LK_query_desc);
           LK_descriptor->compute(ImageGray2,  temp_train,  LK_train_desc);



          cv::Mat distance;
          int k=2; // find two neighbors
          cv::Mat results;
             if(LK_query_desc.type()==CV_8U)
             {
              cv::flann::Index flannIndex(LK_query_desc, cv::flann::LshIndexParams(12, 10 , 2), cvflann::FLANN_DIST_HAMMING);
              flannIndex.knnSearch(LK_train_desc, results, distance, k, cv::flann::SearchParams() );
             }
                 float ErroRatio = 0.5;
             for(unsigned int i=0; i<temp_train.size(); i++)
             {
                 // Apply NNDR
                 if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && distance.at<float>(i,0) <= ErroRatio * distance.at<float>(i,1))
                 {
                     nextPts.push_back(temp_train.at(i).pt);

                     int idx =results.at<int>(i,0);
                     corners.push_back(temp_query.at(idx).pt);
                 }
             }

             distance.release();
             results.release();
             LK_descriptor.release();

             break;
            

         }
      case Freak_descriptor:
            {
                goodFeaturesToTrack(ImageGray1,  LK_query_kpts, 400, 0.001, 8);
                cornerSubPix( ImageGray1, LK_query_kpts, cv::Size(9,9) , cv::Size(-1,-1) , cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.001 ));
                calcOpticalFlowPyrLK(ImageGray1, ImageGray2, LK_query_kpts, LK_train_kpts, status, err, cv::Size(45,45));

                std::vector<cv::KeyPoint> temp_query;

                std::vector<cv::KeyPoint> temp_train;

                //std::vector<cv::KeyPoint> query_kpts;

                Ptr<DescriptorExtractor>  Extractor;

                Extractor= new FREAK(true,true,32.0,1);

                int size_= (int) LK_query_kpts.size();

                for(int i=0;i<size_;i++)

                {
                    temp_query.push_back(cv::KeyPoint(LK_query_kpts[i], 1));
                    temp_train.push_back(cv::KeyPoint(LK_train_kpts[i], 1));
                }

                Extractor->compute(ImageGray1,  temp_query,  LK_query_desc);
                Extractor->compute(ImageGray2,  temp_train,  LK_train_desc);

                //cout<<LK_query_desc.size()<<endl;

                //cv::BFMatcher matcher(cv::NORM_HAMMING2,false);

                cv::Mat distance;
                int k=2; // find the 2 nearest neighbors
                cv::Mat results;
                if(LK_query_desc.type()==CV_8U)
                {
                    cv::flann::Index flannIndex(LK_query_desc, cv::flann::LshIndexParams(12, 10, 2), cvflann::FLANN_DIST_HAMMING/*cvflann::FLANN_DIST_MANHATTAN*/);
                    flannIndex.knnSearch(LK_train_desc, results, distance, k, cv::flann::SearchParams() );
                }
                float nndrRatio = 0.5;
                for(unsigned int i=0; i<temp_train.size(); i++)
                {

                    // Apply NNDR
                    if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && distance.at<float>(i,0) <= nndrRatio * distance.at<float>(i,1))
                    {
                        nextPts.push_back(temp_train.at(i).pt);
                        int idx =results.at<int>(i,0);
                        corners.push_back(temp_query.at(idx).pt);
                    }
                }
                distance.release();
                results.release();
                LK_descriptor.release();
                Extractor.release();

                break;
            }

    }
}
void LKFeatures ::  FeaturesMatched  (std::vector<CvPoint2D32f> &match_query, std::vector<CvPoint2D32f> &match_train)
{
    int size = (int) corners.size();
    for (int i=0;i< size; i++)
    {
        CvPoint2D32f pt_q;
        CvPoint2D32f pt_t;
       pt_q.x = corners[i].x;
       pt_q.y = corners[i].y; 
       pt_t.x = nextPts[i].x;
       pt_t.y = nextPts[i].y;
        
        match_query.push_back(pt_q);
        match_train.push_back(pt_t);
    }  
}
LKFeatures:: ~LKFeatures()
{  
   ImageGray1.release();
   ImageGray2.release();
}

SIFTfeature::SIFTfeature(IplImage* imgGrayA, IplImage* imgGrayB,float Th1, float Th2)
{

    ImageGray1 = cv::cvarrToMat(imgGrayA);
    ImageGray2 = cv::cvarrToMat(imgGrayB);

    th1=Th1;
    th2=Th2;
}

void SIFTfeature::SIFTfeaturematch(vector<CvPoint2D32f> &match_query, vector<CvPoint2D32f> &match_train)
{
    std::vector<cv::KeyPoint> kpts;
    std::vector<cv::KeyPoint> Quepts;

    cv::Mat desc;
    cv::Mat src;

    Ptr<FeatureDetector> siftdet;
    siftdet = new cv::SiftFeatureDetector(0.5f, 10.0f);

    Ptr<DescriptorExtractor> Extractor;
    Extractor= new FREAK(true, true);

    siftdet->detect(ImageGray1, kpts);
    Extractor-> compute(ImageGray1, kpts, desc);
    siftdet->detect(ImageGray2, Quepts);
    Extractor-> compute(ImageGray2, Quepts, src);


    BFMatcher matcher(NORM_HAMMING2, true);
    std::vector<cv::DMatch> vec_matches;
    matcher.match(desc, src, vec_matches);


    for (size_t i = 0; i < vec_matches.size(); ++i)
    {
        const DMatch& match = vec_matches[i];
        CvPoint2D32f pointA;
        CvPoint2D32f pointB;
        pointA.x=kpts[match.queryIdx].pt.x;
        pointA.y=kpts[match.queryIdx].pt.y;

        pointB.x=Quepts[match.trainIdx].pt.x;
        pointB.y=Quepts[match.trainIdx].pt.y;

        match_query.push_back(pointA);
        match_train.push_back(pointB);
    }

    desc.release();
    src.release();

}

SIFTfeature:: ~SIFTfeature()
{
    ImageGray1.release();
    ImageGray2.release();
}


ORBfeature::ORBfeature(IplImage* imgGrayA, IplImage* imgGrayB,float Th1, float Th2)
{
    ImageGray1 = cv::cvarrToMat(imgGrayA);
    ImageGray2 = cv::cvarrToMat(imgGrayB);
}

void ORBfeature::ORBfeaturematch(vector<CvPoint2D32f> &match_query, vector<CvPoint2D32f> &match_train)
{
    std::vector<cv::KeyPoint> kpts;
    std::vector<cv::KeyPoint> Quepts;

    cv::Mat desc;
    cv::Mat src;

    Ptr<FeatureDetector> ORBdet;
    ORBdet=new cv::OrbFeatureDetector(500, 1.0f, 2, 5.0, 1, 0, cv::ORB::FAST_SCORE , 33);
    ORBdet=new cv::OrbFeatureDetector;
    Ptr<DescriptorExtractor> Extractor;
    Extractor= new cv::OrbDescriptorExtractor();

    ORBdet->detect(ImageGray1, kpts);
    cv::KeyPointsFilter::retainBest(kpts, 400);
    Extractor-> compute(ImageGray1, kpts, desc);
    ORBdet->detect(ImageGray2, Quepts);
    cv::KeyPointsFilter::retainBest(Quepts, 400);
    Extractor-> compute(ImageGray2, Quepts, src);
    std::vector<cv::DMatch> vec_matches;

    if(src.type()==CV_8U)
    {
        BFMatcher matcher(NORM_HAMMING, true);
        matcher.match(desc, src, vec_matches);
    }

    for (size_t i = 0; i < vec_matches.size(); ++i)
    {
        const DMatch& match = vec_matches[i];
        CvPoint2D32f pointA;
        CvPoint2D32f pointB;
        pointA.x=kpts[match.queryIdx].pt.x;
        pointA.y=kpts[match.queryIdx].pt.y;

        pointB.x=Quepts[match.trainIdx].pt.x;
        pointB.y=Quepts[match.trainIdx].pt.y;

        match_query.push_back(pointA);
        match_train.push_back(pointB);
    }

    desc.release();
    src.release();

}

ORBfeature:: ~ORBfeature()
{
    ImageGray1.release();
    ImageGray2.release();
}
SURFfeature::SURFfeature(IplImage* imgGrayA, IplImage* imgGrayB,int mTh1)
{
    ImageGray1 = cv::cvarrToMat(imgGrayA);
    ImageGray2 = cv::cvarrToMat(imgGrayB);
    Th1=mTh1;
}

void SURFfeature::SURFfeaturematch(vector<CvPoint2D32f> &match_query, vector<CvPoint2D32f> &match_train)
{
    std::vector<cv::KeyPoint> kpts;
    std::vector<cv::KeyPoint> Quepts;



    SurfFeatureDetector surf (Th1 );
    surf.detect ( ImageGray1 , kpts ) ;

    surf.detect ( ImageGray2 , Quepts ) ;

    // extract descriptors
    SurfDescriptorExtractor fde ;
    cv::Mat desc;
    cv::Mat src;

    fde . compute ( ImageGray1 , kpts , desc ) ;
    fde . compute ( ImageGray2 , Quepts , src );

    //brute−force matching of des criptors
    //BruteForceMatcher<L2<float>> matcher ; //L2 norm
    std::vector<cv::DMatch> vec_matches;
    //matcher. match ( desc , src , vec_matches ) ;

    cv::Mat distance;
    int k=2; // find two neighbors
    cv::Mat results;

    cv::flann::Index flannIndex(desc, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

    // search (nearest neighbor)
    flannIndex.knnSearch(src, results, distance, k, cv::flann::SearchParams() );

    float ErroRatio = 0.8;
    for(unsigned int i=0; i<kpts.size(); i++)
    {
        // Apply NNDR
        //if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && distance.at<float>(i,0) <= ErroRatio * distance.at<float>(i,1))
        //{
            match_query.push_back(kpts.at(i).pt);

            int idx =results.at<int>(i,0);
            match_train.push_back(Quepts.at(idx).pt);
        //}
    }


//    for (size_t i = 0; i < vec_matches.size(); ++i)
//    {
//        const DMatch& match = vec_matches[i];
//        CvPoint2D32f pointA;
//        CvPoint2D32f pointB;
//        pointA.x=kpts[match.queryIdx].pt.x;
//        pointA.y=kpts[match.queryIdx].pt.y;
//
//        pointB.x=Quepts[match.trainIdx].pt.x;
//        pointB.y=Quepts[match.trainIdx].pt.y;
//
//        match_query.push_back(pointA);
//        match_train.push_back(pointB);
//    }

    desc.release();
    src.release();
    

}

SURFfeature:: ~SURFfeature()
{
    ImageGray1.release();
    ImageGray2.release();
}




