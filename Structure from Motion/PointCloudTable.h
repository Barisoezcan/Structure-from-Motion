//File: PointCloudTable.h
#ifndef POINTCLOUDTABLE_H
#define POINTCLOUDTABLE_H




#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "Bildpaar.h"

#include <iostream>
#include <fstream>
#include <map>



using namespace std;


struct measured_3d{
	cv::Point3d XYZ;
	int RGB[3];
	//int num_measurements;
	//vector<image_measurement> observations;
	vector<int> imgs_pnts;
};

extern vector<cv::Point3d> dense_cloud;
extern vector<cv::Vec3b> dense_rgb;


class PointCloudTable{

public:

	//Attribute
	bool initialized = false;


	struct image_measurement{
		int camera_index;
		int feature_index;
		cv::Point2d xy;
	};




	
	int i = 0;


	vector<measured_3d> points3d_measured;


	int bild_anzahl;

	int observations_anzahl = 0;

	vector<int> mask;

	vector<cv::Mat> projection_matrices;
	vector<cv::Mat> projection_matrices_inv;


	map<pair<int, int>, cv::Mat> denseClouds;
	map<pair<int, int>, cv::Mat> denseCloudsColor;




	PointCloudTable(int num_bilder);

	//Methoden
	void push_measurement_init(int, int, cv::Mat, cv::Mat, vector<cv::Point3d>, vector<cv::KeyPoint>, vector<cv::KeyPoint>, vector<cv::DMatch>, map<pair<int, int>, Bildpaar>);
	void push_points3D_2D(int, vector<cv::Point3d>, vector<cv::Point2d>);


	
	void writeAllDenseClouds();

	void find_point3D(vector<cv::Point2d>, vector<cv::Point2d>, vector<cv::Point2d>&, vector<cv::Point3d>&);
	void find_point3D_init(int, vector<cv::KeyPoint>, map<pair<int,int>, Bildpaar>, vector<cv::Point2d>&, vector<cv::Point3d>&);

	void writeHeader(int, int, int);
	void writeCameras(vector<string>, vector<cv::Mat>);

	void push_denseCloud(int _cam1, int _cam2, cv::Mat R1_rect, vector<cv::Point3d> _denseCloud, vector<cv::Vec3b> _denseCloudColor);

	void writePointCloudData(int);

	void writePointCloud(int, int);

	void writeBALProblem(vector<Bild>);

};




#endif