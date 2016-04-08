/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef PNPPROBLEM_H_
#define PNPPROBLEM_H_

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class PnPProblem {

public:
	PnPProblem() { }

	virtual ~PnPProblem();

	void init(const double param[]);  // custom constructor

	void estimatePose(const std::vector<cv::Point3f> &list_points3d,
		const std::vector<cv::Point2f> &list_points2d,
		int flags, bool useExtrinsicGuess);

	cv::Mat get_A_matrix() const { return _A_matrix; }

	cv::Mat get_R_matrix() const { return _R_matrix; }

	cv::Mat get_t_matrix() const { return _t_matrix; }

	cv::Mat get_P_matrix() const { return _P_matrix; }

	cv::Mat get_R_vector() const { return _R_vector; }

	void set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix);

	void set_R_vector(const cv::Mat &R_vector);

private:
	/** The calibration matrix */
	cv::Mat _A_matrix;
	/** The computed rotation matrix */
	cv::Mat _R_matrix;
	/** The computed translation matrix */
	cv::Mat _t_matrix;
	/** The computed projection matrix */
	cv::Mat _P_matrix;
	/** The computed projection matrix */
	cv::Mat _R_vector;
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector
};
#endif /* PNPPROBLEM_H_ */
