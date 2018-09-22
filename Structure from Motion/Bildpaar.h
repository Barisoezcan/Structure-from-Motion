#include "Bild.h"



#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ccalib.hpp>
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include <fstream>



using namespace std;



#pragma once



class Bildpaar{

private:



public:
	//Attribute
	Bild Bild1;
	Bild Bild2;
	int bild1_nr;
	int bild2_nr;
	cv::Mat F;
	cv::Mat E;
	float ratio;
	vector<cv::DMatch> guteMatches;
	vector<cv::Point2d> image_points1, image_points2;

	vector<cv::Point3d> denseCloud;
	vector<cv::Vec3b> denseCloud_color;

	bool isPaar;


	cv::Mat R, t;
	cv::Mat R1_rect;


	bool front;

	//cv::BFMatcher matcher;

	//Funktionen
	Bildpaar();
	Bildpaar(Bild*, Bild*);
	void decomposeEssentialMat2(cv::Mat, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
	bool DecomposeEtoRandT(cv::Mat& E, cv::Mat& R, cv::Mat& t);
	void berechneGuteMatches(cv::Mat, cv::Mat, vector<cv::DMatch>&);
	void berechnePunktpaare(vector<cv::DMatch>, vector<cv::KeyPoint>, vector<cv::KeyPoint>, vector<cv::Point2d>&, vector<cv::Point2d>&);
	void ratioTest(vector<vector<cv::DMatch>>, vector<cv::DMatch>&);
	void symmTest(vector<cv::DMatch>, vector<cv::DMatch>, vector<cv::DMatch>&);
	void ransacFilter(vector<cv::Point2d>, vector<cv::Point2d>, vector<cv::DMatch>, vector<cv::DMatch>&);
	cv::Mat berechneF(vector<cv::Point2d>, vector<cv::Point2d>, vector<cv::DMatch> guteMatches);
	vector<cv::Point3d> triangulate(cv::Mat);
	void berechneE(cv::Mat, cv::Mat, cv::Mat, cv::Mat&);
};