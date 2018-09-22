#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ccalib.hpp>
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2\cudaarithm.hpp"
#include "opencv2\cudafilters.hpp"
#include "opencv2\cudaimgproc.hpp"
#include "opencv2\cudastereo.hpp"
#include "opencv2\cudalegacy.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>

using namespace std;

/*

class StereoMatcher2 {

public:



	//cv::Mat R1;
	 cv::Mat R2;
	 cv::Mat P1;
	 cv::Mat P2;
	 cv::Mat Q;


	 vector<cv::Point3d> colors;
	 vector<cv::Point3d> depth_vec;





public:


	StereoMatcher2();
	void StereoMatcher2::rectify(cv::Mat K, cv::Mat image1, cv::Mat image2, cv::Mat R, cv::Mat t, cv::Mat &P1, cv::Mat &P2, vector<cv::Point3d> &depth_, vector<cv::Vec3b> &colors_, int minDisp);
	//void StereoMatcher::berechneDisparity(cv::Mat, cv::Mat);
	void StereoMatcher2::berechnePointCloud(cv::Mat, vector<cv::Point3d>&, vector<cv::Vec3b>&);
	void StereoMatcher2::writePointCloud();

};

*/