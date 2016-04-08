/*
 * PnPProblem.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#include <iostream>
#include <sstream>
#include "FaceTracker/PnPProblem.h"
#include <opencv2/calib3d/calib3d.hpp>

// Custom constructor given the intrinsic camera parameters
void PnPProblem::init(const double params[])
{
	this->_A_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
	this->_A_matrix.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	this->_A_matrix.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	this->_A_matrix.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	this->_A_matrix.at<double>(1, 2) = params[3];
	this->_A_matrix.at<double>(2, 2) = 1;
	this->_R_matrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
	this->_t_matrix = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
	this->_P_matrix = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix
	this->_R_vector = cv::Mat::zeros(3, 1, CV_64FC1);   // rotation vector
}

PnPProblem::~PnPProblem()
{
	// TODO Auto-generated destructor stub
}

void PnPProblem::set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
	// Rotation-Translation Matrix Definition
	_P_matrix.at<double>(0, 0) = R_matrix.at<double>(0, 0);
	_P_matrix.at<double>(0, 1) = R_matrix.at<double>(0, 1);
	_P_matrix.at<double>(0, 2) = R_matrix.at<double>(0, 2);
	_P_matrix.at<double>(1, 0) = R_matrix.at<double>(1, 0);
	_P_matrix.at<double>(1, 1) = R_matrix.at<double>(1, 1);
	_P_matrix.at<double>(1, 2) = R_matrix.at<double>(1, 2);
	_P_matrix.at<double>(2, 0) = R_matrix.at<double>(2, 0);
	_P_matrix.at<double>(2, 1) = R_matrix.at<double>(2, 1);
	_P_matrix.at<double>(0, 3) = t_matrix.at<double>(0);
	_P_matrix.at<double>(1, 3) = t_matrix.at<double>(1);
	_P_matrix.at<double>(2, 3) = t_matrix.at<double>(2);
}


void PnPProblem::set_R_vector(const cv::Mat &R_vector)
{
	_R_vector.at<double>(0, 0) = R_vector.at<double>(0, 0);
	_R_vector.at<double>(1, 0) = R_vector.at<double>(1, 0);
	_R_vector.at<double>(2, 0) = R_vector.at<double>(2, 0);
}

// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use
void PnPProblem::estimatePose(const std::vector<cv::Point3f> &list_points3d,		// list with model 3D coordinates
	const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
	int flags, bool useExtrinsicGuess)
{
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients

	cv::solvePnP(list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

	Rodrigues(rvec, _R_matrix);      // converts Rotation Vector to Matrix
	_t_matrix = tvec;				  // set translation matrix

	this->set_P_matrix(_R_matrix, _t_matrix); // set rotation-translation matrix
	this->set_R_vector(rvec);					// set rotation vector
}
