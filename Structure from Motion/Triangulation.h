#include "Bildpaar.h"
//#include "PointCloudTable.h"



class Triangulation{

public:
	//Attribute
	vector<Bildpaar> bildpaare;
	//PointCloudTable pct;
	vector<cv::Matx34d> projection_matrices;

	cv::Matx34d P0_old, P1_old;
	cv::Mat_<cv::Vec3f> image3d;

	//Funktionen
	Triangulation();
	//Triangulation(Bildpaar);
	//Triangulation(vector<Bildpaar>);
	void insertImage(Bildpaar);

	cv::Mat image1_rect, image2_rect;

	cv::Mat depth;
	vector<cv::Point3d> depth_vec;

	cv::Mat Q;


	vector<cv::Point3d> initialize_model(Bildpaar, cv::Mat&, cv::Mat&);




	vector<cv::Point3d> LinearTriangulation(cv::Mat, cv::Mat, cv::Mat, vector<cv::Point2d>, vector<cv::Point2d>);


	cv::Mat_<double> IterativeLinearLSTriangulation(cv::Matx34d, cv::Matx34d, cv::Point3d, cv::Point3d);
	cv::Point3d LinearLSTriangulation(cv::Matx34d, cv::Matx34d, cv::Point3d, cv::Point3d);
	int disambiguateCameraPose(vector<cv::Point3d>, cv::Mat, cv::Mat);
	void NonLinearTriangulation(cv::Mat, cv::Mat, cv::Mat, cv::Mat, cv::Mat, vector<cv::DMatch>, vector<cv::DMatch>, vector<cv::Point3d>);
	void writePointCloud(vector<cv::Point3d>, int, string);
	void writePointCloudEnd(int);
	void writeCameras(vector<cv::Point3d>, int);
	void writeDisp(int);

	void berechneDisparity();
	void berechnePointCloud();


	cv::Mat image_disparity_RAW;
	cv::Mat image_disparity_RAW_CV32F;
	cv::Mat image_disparity_RAW_CV32F_norm;
	cv::Mat image_disparity_RAW_CV32F_div16;


	int minDisparity = 160;
	int numDisparities = 5 * 16;
	int blockSize = 5;
	int P1 = 8;
	int P2 = 32;
	int disp12MaxDiff = 1;
	int preFilterCap = 1;
	int uniquenessRatio = 10;
	int speckleWindowSize = 100;
	int speckleRange = 32;
	int mode = 0;



	cv::Mat R1, R2, P1_rect, P2_rect;








	cv::Ptr<cv::StereoSGBM> sgbm;


	//ofstream myfile;


};