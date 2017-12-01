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

    bool FindFiducial_ARUCO(cv::Mat image);

    // bool FindFiducial_SURF(cv::Mat image);

    bool FindFiducial_Contours(cv::Mat image);

    void SetFiducialTemplate(cv::Mat templateImage)
       { m_templateImage = templateImage; };

protected:

    cv::Mat Filter();
    cv::Mat Blur();
    cv::Mat Sharpen();

    cv::Ptr<cv::aruco::Dictionary> GetArucoF(bool filled);

    void Show(cv::Mat img);
  
    cv::Mat m_templateImage;

};


#endif /* fiducialfinder_hpp */
