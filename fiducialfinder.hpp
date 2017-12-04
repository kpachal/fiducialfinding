//
//  fiducialfinder.hpp
//  fiducialfinding
//
//  Created by Katherine Pachal on 30.11.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#ifndef fiducialfinder_hpp
#define fiducialfinder_hpp

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco.hpp>

class FiducialFinder
{
public:
	FiducialFinder();
	~FiducialFinder();

    bool FindFiducial_ARUCO(cv::Mat image, bool fiducialIsFilled = false, std::string name = "");

    // bool FindFiducial_SURF(cv::Mat image);

    bool FindFiducial_Contours(cv::Mat image, bool doTemplate = false);

    void SetFiducialTemplate(cv::Mat templateImage);

protected:

    cv::Mat Filter(cv::Mat img);
    cv::Mat Blur(cv::Mat img);
    cv::Mat Sharpen(cv::Mat img);

    cv::Ptr<cv::aruco::Dictionary> GetArucoF(bool filled = false);
  

    void Show(cv::Mat img);
    void DrawLine (cv::Mat img, cv::Point2f start, cv::Point2f end);
    void DrawCircle (cv::Mat img, cv::Point2f point, int radius );
  
    cv::Mat m_templateImage;
  
    double m_pixToMicron;
  
    cv::RNG m_rng;
  
    cv::Moments m_templateMu;
    std::vector<double> m_templateHu;

};


#endif /* fiducialfinder_hpp */
