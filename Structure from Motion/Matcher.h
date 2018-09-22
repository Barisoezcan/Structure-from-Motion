#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ccalib.hpp>



using namespace std;



struct matching_pair{
	int image_query;
	int image_train;
	vector<cv::KeyPoint> keypoints1;
	vector<cv::KeyPoint> keypoints2;
	vector<cv::Point2f> points1;
	vector<cv::Point2f> points2;
	vector<cv::DMatch> matches;
	cv::Mat F;
};





void goodMatcher(vector<cv::Mat> images, vector<vector<cv::KeyPoint>> &keypoints, vector<cv::Mat> &descriptors, vector<cv::DMatch> &matches, vector<matching_pair> &pairs);

void ratioTest(vector<vector<cv::DMatch>> &matches);

void symmTest(vector<vector<cv::DMatch>> matches1_temp, vector<vector<cv::DMatch>> matches2_temp, vector<cv::DMatch> &symmMatches);

void symmTest2(vector<cv::DMatch> matches1_temp, vector<cv::DMatch> matches2_temp, vector<cv::DMatch> &symmMatches);