#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ccalib.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include "opencv2/xfeatures2d/cuda.hpp"


#include "opencv2/imgproc/detail/distortion_model.hpp"

#include "cudaImage.h"
#include "cudaSift.h"


#pragma once


using namespace std;
using namespace cv::xfeatures2d;




class Bild{

private:



public:
	int Bild_nr;
	cv::Mat image_distorted;
	cv::Mat image;
	cv::Mat image_gray;
	cv::cuda::GpuMat imageGPU;
	cv::Mat K;
	cv::Mat distCoeffs;
	cv::Mat P;

	vector<cv::KeyPoint> keypoints;
	cv::cuda::GpuMat keypointsGPU;
	vector<cv::Point2d> feature_points;
	cv::cuda::GpuMat descriptorsGPU;
	//vector<float> descriptors;
	cv::Mat descriptors;

	int minHessian = 400;
	//cv::Ptr<cv::xfeatures2d::SURF> detector;
	//Ptr<FastFeatureDetector> detector2;
	cv::cuda::SURF_CUDA surf;

	Bild();
	Bild(int, cv::Mat, cv::Mat, cv::Mat);
	void berechneKeypoints();
	void berechneDescriptors();
	void berechneFeaturePoints();


	SiftData siftData;

};