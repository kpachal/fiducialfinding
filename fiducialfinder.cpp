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
#include <math.h>

FiducialFinder::FiducialFinder() :
m_rng(12345) {

  cv::namedWindow("display",CV_WINDOW_NORMAL);
  m_pixToMicron = 3.0;

}

FiducialFinder::~FiducialFinder() {

  cv::destroyAllWindows();

}

bool FiducialFinder::FindFiducial_ARUCO(cv::Mat image, bool fiducialIsFilled, std::string name) {
  
  // This unmodified version will hold the detected markers
  cv::Mat outputImage = image;

  // Set up parameters for marker drawing
  cv::Ptr<cv::aruco::Dictionary> dictionary = GetArucoF(fiducialIsFilled);

  cv::Mat markerImg;
  cv::aruco::drawMarker(dictionary, 0, 200, markerImg, 1);

  // Let's look at the marker!
  //Show(markerImg);
  
  // Now check that we have something to search it in
  Show(image);
  
  // Use dimension of image in pixels to find where our
  // field of view is centered.
  int center_x = image.size().width/2.0;
  int center_y = image.size().height/2.0;
  cv::Point2f center_point(center_x,center_y);
  std::cout << "Center of image is at " << center_x << ", " << center_y << std::endl;
  
  // Find fiducial in image
  std::vector< int > markerIds;
  std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  parameters->perspectiveRemovePixelPerCell = 1;
  parameters->perspectiveRemoveIgnoredMarginPerCell = 0.3;
  parameters->errorCorrectionRate = 0.3;
//  parameters->adaptiveThreshWinSizeMin = 10;
//  parameters->adaptiveThreshWinSizeMax = 100;
//  parameters->adaptiveThreshWinSizeStep = 10;
  
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  std::cout << "Beginning loop" << std::endl;
  for (int mark = 0; mark < markerIds.size(); mark++) {

    // How far is it to the fiducial?
    std::vector<cv::Point2f> corners = markerCorners.at(mark);
    
//    for (int corner=0; corner < corners.size(); corner++) {
//      std::cout << "Corner " << corner << " is at ";
//      int x = corners.at(corner).x;
//      int y = corners.at(corner).y;
//      std::cout << x << " " << y << std::endl;
//    }
    int x = corners.at(0).x;
    int y = corners.at(0).y;
      
    // Corners begin indexing from top left.
    // We want bottom left I guess? Maybe we can do either.
    // For now, use corner #1 to estimate distance.
    int distance_pix_x = x - center_x;
    int distance_pix_y = y - center_y;
    
    float distance_microns = sqrt(pow(distance_pix_x,2) + pow(distance_pix_y,2))/m_pixToMicron;
  
    std::cout << "The fiducial is " << distance_microns << " microns from the center of the camera. " << std::endl;
    std::cout << "Move by " << (center_x - x)/m_pixToMicron << ", ";
    std::cout << -1*(center_y - y)/m_pixToMicron << " to align." << std::endl;
  
    // Draw that line on the image.
    DrawLine(image, center_point, corners.at(0));
    DrawCircle (image, center_point, 4 );
  
    // What angle is the fiducial at?
    // 0 degrees would have the F exactly up and down.
    
  
  }

  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
  Show(outputImage);
  
  if (!name.empty()) imwrite(name, outputImage);
  
  bool foundFiducial = markerIds.size() > 0 ? true : false;
  return foundFiducial;

}

bool FiducialFinder::FindFiducial_Contours(cv::Mat image) {

  // Check that we have something to search in
  Show(image);
  
  int kernel_size = 3;
  cv::Mat temp;
  double otsu_thresh_val = cv::threshold(image, temp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  int lowThreshold = 0.75*otsu_thresh_val;
  int highThreshold = otsu_thresh_val;
  // Or roughly 100, 300?
  
  cv::Mat smoothed = Filter(image);
  
  // Apply Canny edge detection
  cv::Mat edges;
  Canny(smoothed, edges, lowThreshold, highThreshold, kernel_size);
  
  // Now pick out contours from this.
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  // Draw the contours we found.
  cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ ) {
       cv::Scalar color = cv::Scalar(m_rng.uniform(0, 255), m_rng.uniform(0,255), m_rng.uniform(0,255));
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }
  Show(drawing);

  // Now search through contours.
  std::vector<std::vector<cv::Point> > contours_keep;
  for( int i = 0; i< contours.size(); i++ ) {
  
    std::vector<cv::Point> cont = contours.at(i);
    cv::Vec4i hier = hierarchy.at(i);
  
    // Want to filter out non-closed contours
    //Check if there is a child contour :
    if(hier[2]<0) continue;

    // Want to filter out all the uninteresting contours
    // that are really just noise. Can do this by size,
    // since even the really long ones often have near
    // zero size.
//    double perimeter = arcLength(cont,true);
//    double size = contourArea(cont,true);
    
//    if (size < image.size().width/1000.0) continue;

//    if (perimeter < image.size().width/250.0 || size < image.size().width/250.0) continue;
    
    // Check how F-like remaining curves are
//    approx = cv2.approxPolyDP(c, 0.02 * peri, True)

  
    // Keep perimeters we want only
    contours_keep.push_back(cont);
  }


  // Draw the contours we found.
  drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
  for( int i = 0; i< contours_keep.size(); i++ ) {
       cv::Scalar color = cv::Scalar(m_rng.uniform(0, 255), m_rng.uniform(0,255), m_rng.uniform(0,255));
       drawContours( drawing, contours_keep, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

  // What did we get?
  Show(drawing);

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
 
  return ptrDict;

}


void FiducialFinder::Show(cv::Mat img) {

  cv::imshow("display",img);
  cv::waitKey(0);

}

void FiducialFinder::DrawLine (cv::Mat img, cv::Point2f start, cv::Point2f end ) {
  
  int thickness = 2;
  int lineType = 8;
  line( img,
        start,
        end,
        cv::Scalar( 255, 0, 0 ),
        thickness,
        lineType );
}

void FiducialFinder::DrawCircle (cv::Mat img, cv::Point2f point, int radius ) {
  
  int thickness = 2;
  int lineType = 8;

  circle(img,
         point,
         radius,
         cv::Scalar( 255, 0, 0 ), thickness, lineType, 0);
}

cv::Mat FiducialFinder::Filter(cv::Mat img) {

  cv::Mat out;
//  medianBlur(img,out,3);
  cv::bilateralFilter(img, out, 11,17,17);
  return out;

}

cv::Mat FiducialFinder::Blur(cv::Mat img) {

  cv::Mat out;
  GaussianBlur(img,out,cv::Size(7,7),1.5,1.5,cv::BORDER_DEFAULT);

  return out;

}

cv::Mat FiducialFinder::Sharpen(cv::Mat img) {

  cv::Mat out;
  
  
  return out;

}
