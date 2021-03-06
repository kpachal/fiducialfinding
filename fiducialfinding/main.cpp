//
//  main.cpp
//  katetest
//
//  Created by Katherine Pachal on 27.10.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#include <iostream>
#include <iomanip>

#include "opencv2/opencv.hpp"

#include "FocusFinder.hpp"
#include "fiducialfinder.hpp"

using namespace cv;
int main(int, char**)
{

    bool testContours = true;

    // Make autofocus device
    FiducialFinder myFinder;

    // Name of image to use as fiducial template, if relevant
    std::string fiducialTemplate = "";
  
    // Need it for contour testing; take it from old photos
    if (testContours)
      fiducialTemplate = "/Users/kpachal/Code/ITk/FiducialTesting/R03_fiducial.jpg";

    cv::Mat templateImage;
    if (!fiducialTemplate.empty()) {
      templateImage = imread(fiducialTemplate, CV_LOAD_IMAGE_GRAYSCALE);
      myFinder.SetFiducialTemplate(templateImage);
    }
  
    // For aruco tests,
    // Going to look at each photo from Carleton and determine which ones contain fiducials
    for (int i=1; i< 13; i++) {
    
      if (testContours) continue;

      // No img. no 6
      if (i==6) continue;
      
      // Just for speed...
      if (i < 5 || i > 8 ) continue;

      std::stringstream ss;
      ss << std::setfill ('0') << std::setw (4);
      ss << i ;
      std::string numstring = ss.str();

      // Name of image in which to seek fiducial
      std::string imgFile = "/Users/kpachal/Code/ITk/FiducialTesting/photosFromCarleton/SFUs-A07-mini_bz2-p2-";
      imgFile = imgFile + numstring + ".png";
      std::cout << "Getting " << imgFile << std::endl;
  
      // Load images and convert to grayscale
      cv::Mat searchImage = imread(imgFile, 0);

      // Carleton's camera has 2.5x the resolution of ours. Scale down,
      // and then again by a bit more so I can see it.
      resize(searchImage, searchImage, cv::Size(), 1.0/2.5, 1.0/2.5);
      resize(searchImage, searchImage, cv::Size(), 0.5, 0.5);

      std::string outName = "/Users/kpachal/Documents/Slides/ITk/VancouverMeetings/FiducialFinding_2017.12.05/my_images/CarletonPic_"+numstring+".png";
      bool gotAruco = myFinder.FindFiducial_ARUCO(searchImage,true,outName);
  
      //bool gotContours = myFinder.FindFiducial_Contours(searchImage);
  
      std::cout << "In image " << numstring << ": " << std::endl;
      std::cout << "gotAruco is " << gotAruco << std::endl;
//      std::cout << "gotContours is " << gotContours << std::endl;
    }
  
    // For contour tests,
    // going to look at each photo of our electric mini-sensors and try to pick
    // out the F compared to our template image.
    for (int i=3; i<8; i++) {
    
      // No img. no 6
      if (i==6) continue;
      
      std::stringstream ss;
      ss << std::setfill ('0') << std::setw (2);
      ss << i ;
      std::string numstring = ss.str();

      // Name of image in which to seek fiducial
      std::string imgFile = "/Users/kpachal/Code/ITk/FiducialTesting/R";
      imgFile = imgFile + numstring + ".tif";
      std::cout << "Getting " << imgFile << std::endl;

      // Load images and convert to grayscale
      cv::Mat searchImage = imread(imgFile, 0);

      bool gotContours = myFinder.FindFiducial_Contours(searchImage);

      std::cout << "In image " << numstring << ": " << std::endl;
      std::cout << "gotContours is " << gotContours << std::endl;
      
    }

    return 0;
}
