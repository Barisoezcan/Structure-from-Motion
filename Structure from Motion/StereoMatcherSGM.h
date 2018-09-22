#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ccalib.hpp>
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudastereo.hpp"
#include "opencv2/cudalegacy.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>
#include "SGM_Wrapper.h"

using namespace std;
using namespace cv;



class StereoMatcherSGM
{
public:
	int z = 0;

	//Mat P1, P2;
	//Mat R1;
	Mat R2;
	Mat Q;

	int minDisp, maxDisp;
	Mat image1_rect, image2_rect;
	Mat gray_image1, gray_image2;
	vector<Mat> image1_roi, image2_roi;


	//Ptr<StereoBM> bm;
	int minDisparity;
	int numDisparities;
	int blockSize;


	Mat image_disparity;
	Mat image_disparity_CV32F;
	Mat depth;

	vector<Vec3b> colors;
	vector<Point3d> depth_vec;

public:
	StereoMatcherSGM();
	~StereoMatcherSGM();
	Mat translateImg(Mat &img, int offsetx, int offsety);
	void StereoMatcherSGM::rectify(Mat K, Mat image1, Mat image2, Mat R, Mat t, Mat& image1_rect, Mat& image2_rect, Mat &R1, Mat &P1, Mat &P2, Mat& Q, int minx1, int minx2, int miny1, int miny2);
	void berechneDisparity(Mat& image1_rect, Mat& image2_rect);
	void StereoMatcherSGM::berechnePointCloud(Mat& image1, Mat& image2, Mat K, Mat R, Mat t, Mat& R1, Mat& P1, Mat& P2, vector<Point3d>& _depth, vector<Vec3b>& _colors, int minx1, int minx2, int miny1, int miny2);
	void writePointCloud();

};

