//
//  fiducialfinder.cpp
//  fiducialfinding
//
//  Created by Katherine Pachal on 30.11.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#include "fiducialfinder.hpp"
#include <utility>
#include <functional>

FiducialFinder::FiducialFinder() {

  cv::namedWindow("display",CV_WINDOW_NORMAL);

}

FiducialFinder::~FiducialFinder() {

  cv::destroyAllWindows();

}

bool FiducialFinder::FindFiducial_ARUCO(cv::Mat image) {

  // Set up parameters for marker drawing
  cv::Ptr<cv::aruco::Dictionary> dictionary = GetArucoF(false);

  cv::Mat markerImg;
  cv::aruco::drawMarker(dictionary, 0, 200, markerImg, 1);

  // Let's look at the marker!
  Show(markerImg);
  
  // Now check that we have something to search it in
  Show(image);

  return false;

}

bool FiducialFinder::FindFiducial_Contours(cv::Mat image) {




  return false;
  
}

cv::Ptr<cv::aruco::Dictionary> FiducialFinder::GetArucoF(bool filled) {

  cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::Dictionary::create(0,6);
  
  cv::Mat markerBits = (cv::Mat_<int>(6,6,CV_8UC1) << 0, 0, 0, 0, 0, 0,
                                                      0, 1, 1, 1, 1, 0,
                                                      0, 1, 0, 0, 0, 0,
                                                      0, 1, 1, 1, 0, 0,
                                                      0, 1, 0, 0, 0, 0,
                                                      0, 1, 0, 0, 0, 0);
  
  if (filled) markerBits = (markerBits & 0);
  
  std::cout << "M = " << std::endl << " "  << markerBits << std::endl << std::endl;

  cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits);

  dictionary->bytesList.push_back(markerCompressed);
  
  cv::Mat markerImg;
  cv::aruco::drawMarker(dictionary, 0, 8, markerImg, 1);
  
  std::cout << "new M = " << std::endl << " "  << markerImg << std::endl << std::endl;

 
  return dictionary;

}

void FiducialFinder::Show(cv::Mat img) {

  cv::imshow("display",img);
  cv::waitKey(0);

}
