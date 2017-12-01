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

bool FiducialFinder::FindFiducial_ARUCO(cv::Mat image, bool fiducialIsFilled) {

  // Set up parameters for marker drawing
  cv::Ptr<cv::aruco::Dictionary> dictionary = GetArucoF(fiducialIsFilled);

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

  cv::aruco::Dictionary dictionary;
  dictionary.markerSize = 6;
  dictionary.maxCorrectionBits = 12;

  bool bkg = 1;
  bool F = 0;
  if (filled) {
    bkg = 0; F = 1;
  }

  cv::Mat markerBits(6,6,CV_8UC1,cv::Scalar(bkg));
  markerBits.at<bool>(1,1) = F;
  markerBits.at<bool>(1,2) = F;
  markerBits.at<bool>(1,3) = F;
  markerBits.at<bool>(1,4) = F;
  markerBits.at<bool>(2,1) = F;
  markerBits.at<bool>(3,1) = F;
  markerBits.at<bool>(3,2) = F;
  markerBits.at<bool>(3,3) = F;
  markerBits.at<bool>(4,1) = F;
  markerBits.at<bool>(5,1) = F;
  
  cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(markerBits);
  dictionary.bytesList.push_back(markerCompressed);
  cv::Ptr<cv::aruco::Dictionary> ptrDict = cv::makePtr<cv::aruco::Dictionary>(dictionary);
  
  cv::Mat markerImg;
  cv::aruco::drawMarker(ptrDict, 0, 8, markerImg, 1);
 
  return ptrDict;

}

void FiducialFinder::Show(cv::Mat img) {

  cv::imshow("display",img);
  cv::waitKey(0);

}
