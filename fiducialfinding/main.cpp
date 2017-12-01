//
//  main.cpp
//  katetest
//
//  Created by Katherine Pachal on 27.10.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#include <iostream>
#include "opencv2/opencv.hpp"

#include "FocusFinder.hpp"
#include "fiducialfinder.hpp"

using namespace cv;
int main(int, char**)
{

    // Name of image in which to seek fiducial
    std::string imgFile = "/Users/kpachal/Code/ITk/FiducialTesting/photosFromCarleton/SFUs-A07-mini_bz2-p2-0005.png";
  
    // Name of image to use as fiducial template, if relevant
    std::string fiducialTemplate = "";
  
    // Load images and convert to grayscale
    cv::Mat searchImage = imread(imgFile, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat templateImage;
    if (!fiducialTemplate.empty())
      templateImage = imread(fiducialTemplate, CV_LOAD_IMAGE_GRAYSCALE);

    // Make autofocus device
    FiducialFinder myFinder;

    // If relevant: load template image to define fiducial
    if (!fiducialTemplate.empty()) myFinder.SetFiducialTemplate(templateImage);

    bool gotAruco = myFinder.FindFiducial_ARUCO(searchImage);
  
    bool gotContours = myFinder.FindFiducial_Contours(searchImage);
  
    std::cout << "gotAruco is " << gotAruco << std::endl;
    std::cout << "gotContours is " << gotContours << std::endl;

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
