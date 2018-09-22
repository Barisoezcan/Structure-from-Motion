#include "Triangulation.h"


#include <opencv2/highgui/highgui.hpp>


/*
Triangulation::Triangulation(Bildpaar _bildpaar)
{
	Matx34d P_init, P1;


	//Initialisiere PointCloudTable
	if (pct.points3d_measured.size() == 0)
	{
		initialize_model(_bildpaar, P_init, P1);
	}
}
*/

Triangulation::Triangulation(){}





vector<cv::Point3d> Triangulation::initialize_model(Bildpaar _bildpaar, cv::Mat &P_init, cv::Mat &P1)
{
	//Trianguliere die initialen Bildpaare
	P_init = (cv::Mat_<double>(4,4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0, 
		0, 0, 0, 1);

	P1 = (cv::Mat_<double>(4,4) <<
		_bildpaar.R.at<double>(0, 0), _bildpaar.R.at<double>(0, 1), _bildpaar.R.at<double>(0, 2), _bildpaar.t.at<double>(0),
		_bildpaar.R.at<double>(1, 0), _bildpaar.R.at<double>(1, 1), _bildpaar.R.at<double>(1, 2), _bildpaar.t.at<double>(1),
		_bildpaar.R.at<double>(2, 0), _bildpaar.R.at<double>(2, 1), _bildpaar.R.at<double>(2, 2), _bildpaar.t.at<double>(2),
		0, 0, 0, 1);


	vector<cv::Point3d> points3d = LinearTriangulation(_bildpaar.Bild1.K, P_init, P1, _bildpaar.image_points1, _bildpaar.image_points2);


	return points3d;
}








/*
Triangulation::Triangulation(vector<Bildpaar> _bildpaare)
{
	bildpaare = _bildpaare;


	Matx34d P_init(	1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0);

	Matx34d P1(bildpaare.at(0).R.at<double>(0, 0), bildpaare.at(0).R.at<double>(0, 1), bildpaare.at(0).R.at<double>(0, 2), bildpaare.at(0).t.at<double>(0),
		bildpaare.at(0).R.at<double>(1, 0), bildpaare.at(0).R.at<double>(1, 1), bildpaare.at(0).R.at<double>(1, 2), bildpaare.at(0).t.at<double>(1),
		bildpaare.at(0).R.at<double>(2, 0), bildpaare.at(0).R.at<double>(2, 1), bildpaare.at(0).R.at<double>(2, 2), bildpaare.at(0).t.at<double>(2));



	vector<Point3d> points3d = LinearTriangulation(bildpaare.at(0).Bild1.K, P_init, P1, bildpaare.at(0).image_points1, bildpaare.at(0).image_points2);


	vector<Point3d> points3d_init;


	points3d_init = points3d;



	pct.projection_matrices.push_back(P_init);
	pct.projection_matrices.push_back(P1);
	


	P0_old = P_init;
	P1_old = P1;

	pct.push_points3D_2D_init(0, points3d_init, bildpaare.at(0).image_points2);



	sgbm = StereoSGBM::create(0, 16, 3);



	stereoRectify(bildpaare.at(0).Bild1.K, Mat(), bildpaare.at(0).Bild2.K, Mat(), bildpaare.at(0).Bild2.image.size(), bildpaare.at(0).R, bildpaare.at(0).t, R1, R2, P1_rect, P2_rect, Q);


	Mat_<float> Q_float;
	Q.copyTo(Q_float);




	Mat mapx1, mapy1, mapx2, mapy2;
	initUndistortRectifyMap(bildpaare.at(0).Bild1.K, Mat(), R1, P1_rect, bildpaare.at(0).Bild1.image.size(), CV_32FC1, mapx1, mapy1);
	initUndistortRectifyMap(bildpaare.at(0).Bild2.K, Mat(), R2, P2_rect, bildpaare.at(0).Bild2.image.size(), CV_32FC1, mapx2, mapy2);



	remap(bildpaare.at(0).Bild1.image, image1_rect, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, 0);
	remap(bildpaare.at(0).Bild2.image, image2_rect, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, 0);




	//sgbm->setP1(8);
	//sgbm->setP2(32);
	//sgbm->setMinDisparity(16);
	//sgbm->setNumDisparities(160);
	//sgbm->setUniquenessRatio(10);
	//sgbm->setSpeckleWindowSize(100);
	//sgbm->setSpeckleRange(32);
	//sgbm->setDisp12MaxDiff(1);



	//Mat disp;
	//sgbm->compute(image1_rect, image2_rect, disp);


	//
	//reprojectImageTo3D(disp, image3d, Q);


	//Mat_<Vec3f> XYZ(disp.rows, disp.cols);   // Output point cloud
	//Mat_<float> vec_tmp(4, 1);

	//cout << disp.size();


	

	
	for (int y = 0; y < disp.rows; ++y) 
	{
		for (int x = 0; x < disp.cols; ++x) 
		{
			vec_tmp(0) = x; 
			vec_tmp(1) = y; 
			vec_tmp(2) = disp.at<float>(y, x); 
			vec_tmp(3) = 1;

			vec_tmp = Q_float * vec_tmp;

			vec_tmp /= vec_tmp(3);

			Vec3f &point = XYZ.at<Vec3f>(y, x);

			point[0] = vec_tmp(0);
			point[1] = vec_tmp(1);
			point[2] = vec_tmp(2);
		}
		cout << "row: " << y << endl;
	}

	







	berechneDisparity();




	berechnePointCloud();













	//XYZ.copyTo(image3d);



	writePointCloudEnd(0);


	Matx34d P0_temp;

	for (int i = 1; i < bildpaare.size(); i++)
	{
		cout << "Bild1_nr: " << i << endl;


		P0_temp = P1_old;

		vector<Point2d> im2_points2d;
		vector<Point3d> im2_points3d;

		Mat mask;

		pct.find_point3D(bildpaare.at(i).image_points1, bildpaare.at(i).image_points2, im2_points2d, im2_points3d);



		Mat r_rog, t;


		solvePnPRansac(im2_points3d, im2_points2d, bildpaare.at(i).Bild1.K, Mat(), r_rog, t);

		Mat_<double> R_rod(3, 3);
		Mat_<double> t1(t);

		Rodrigues(r_rog, R_rod);



		Matx34d P1_temp(R_rod(0, 0), R_rod(0, 1), R_rod(0, 2), t1(0),
			R_rod(1, 0), R_rod(1, 1), R_rod(1, 2), t1(1),
			R_rod(2, 0), R_rod(2, 1), R_rod(2, 2), t1(2));


		vector<Point3d> points3d_temp = LinearTriangulation(bildpaare.at(i).Bild1.K, P0_temp, P1_temp, bildpaare.at(i).image_points1, bildpaare.at(i).image_points2);

		P1_old = P1_temp;


		pct.push_points3D_2D(i, points3d_temp, bildpaare.at(i).image_points2);
		pct.projection_matrices.push_back(P1_old);




		//pct.projection_matrices.push_back(P1_temp_.inv());

		writePointCloudEnd(bildpaare.at(i).Bild1.Bild_nr);





	}







}

















void Triangulation::insertImage(Bildpaar _bildpaar)
{
	bildpaare.push_back(_bildpaar);

	Matx34d P0_temp;

	P0_temp = P1_old;

	vector<Point2d> im2_points2d;
	vector<Point3d> im2_points3d;

	pct.find_point3D(_bildpaar.image_points1, _bildpaar.image_points2, im2_points2d, im2_points3d);


	Mat r_rog, t;
	solvePnP(im2_points3d, im2_points2d, _bildpaar.Bild1.K, Mat(), r_rog, t, false);

	Mat_<double> R_rod(3, 3);
	Mat_<double> t1(t);

	Rodrigues(r_rog, R_rod);



	Matx34d P1_temp(	R_rod(0, 0), R_rod(0, 1), R_rod(0, 2), t1(0),
						R_rod(1, 0), R_rod(1, 1), R_rod(1, 2), t1(1),
						R_rod(2, 0), R_rod(2, 1), R_rod(2, 2), t1(2));


	cout << "P0: " << P0_temp << endl << endl;
	cout << "P1: " << P1_temp << endl << endl << endl;


	vector<Point3d> points3d_temp = LinearTriangulation(_bildpaar.Bild1.K, P0_temp, P1_temp, _bildpaar.image_points1, _bildpaar.image_points2);




	pct.push_points3D_2D(0, points3d_temp, _bildpaar.image_points2);



	

	//pct.projection_matrices.push_back(P1_temp_.inv());

	writePointCloud(pct.points3d, _bildpaar.Bild1.Bild_nr, "Beton");




}

*/






vector<cv::Point3d> Triangulation::LinearTriangulation(cv::Mat K, cv::Mat _P1, cv::Mat _P2, vector<cv::Point2d> image_points1, vector<cv::Point2d> image_points2)
{
	cv::Matx34d P1(
		_P1.at<double>(0, 0), _P1.at<double>(0, 1), _P1.at<double>(0, 2), _P1.at<double>(0, 3),
		_P1.at<double>(1, 0), _P1.at<double>(1, 1), _P1.at<double>(1, 2), _P1.at<double>(1, 3),
		_P1.at<double>(2, 0), _P1.at<double>(2, 1), _P1.at<double>(2, 2), _P1.at<double>(2, 3));

	cv::Matx34d P2(
		_P2.at<double>(0, 0), _P2.at<double>(0, 1), _P2.at<double>(0, 2), _P2.at<double>(0, 3),
		_P2.at<double>(1, 0), _P2.at<double>(1, 1), _P2.at<double>(1, 2), _P2.at<double>(1, 3),
		_P2.at<double>(2, 0), _P2.at<double>(2, 1), _P2.at<double>(2, 2), _P2.at<double>(2, 3));

	vector<cv::Point3d> points3d_temp;

	cv::Mat K_inv = K.inv();

	for (int i = 0; i < image_points1.size(); i++)
	{
		cv::Point3d point1_h(image_points1.at(i).x, image_points1.at(i).y, 1);
		cv::Point3d point2_h(image_points2.at(i).x, image_points2.at(i).y, 1);



		cv::Mat_<double> point1_h_mapping = K_inv * cv::Mat_<double>(point1_h);
		cv::Mat_<double> point2_h_mapping = K_inv * cv::Mat_<double>(point2_h);



		cv::Point3d point3D1(point1_h_mapping(0), point1_h_mapping(1), point1_h_mapping(2));
		cv::Point3d point3D2(point2_h_mapping(0), point2_h_mapping(1), point2_h_mapping(2));


		cv::Mat_<double> point_temp_Mat = IterativeLinearLSTriangulation(P1, P2, point3D1, point3D2);

		cv::Point3d point_temp(point_temp_Mat(0), point_temp_Mat(1), point_temp_Mat(2));

		points3d_temp.push_back(point_temp);
		//points2d_temp.push_back(image_points2.at(i));
	}
	return points3d_temp;

}




cv::Mat_<double> Triangulation::IterativeLinearLSTriangulation(cv::Matx34d P0, cv::Matx34d P1, cv::Point3d u0, cv::Point3d u1)
{
	double wi = 1, wi1 = 1;

	cv::Mat_<double> X(4, 1);

	cv::Point3d X_temp = LinearLSTriangulation(P0, P1, u0, u1);


	cv::Mat_<double> X_(4, 1);

	X_ = cv::Mat_<double>(X_temp);


	X(0) = X_(0); 
	X(1) = X_(1); 
	X(2) = X_(2); 
	X(3) = 1.0;

	for (int i = 0; i < 10; i++)
	{


		//recalculate weights
		double p2x = cv::Mat_<double>(cv::Mat_<double>(P0).row(2)*X)(0);
		double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		//if (fabsf(wi - p2x) <= .1 && fabsf(wi1 - p2x1) <= .1) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		cv::Mat A = (cv::Mat_<double>(4, 3) <<	(u0.x*P0(2, 0) - P0(0, 0)) / wi, (u0.x*P0(2, 1) - P0(0, 1)) / wi, (u0.x*P0(2, 2) - P0(0, 2)) / wi,
										(u0.y*P0(2, 0) - P0(1, 0)) / wi, (u0.y*P0(2, 1) - P0(1, 1)) / wi, (u0.y*P0(2, 2) - P0(1, 2)) / wi,
										(u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
										(u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1);

		cv::Mat B = (cv::Mat_<double>(4, 1) <<	-(u0.x*P0(2, 3) - P0(0, 3)) / wi,
										-(u0.y*P0(2, 3) - P0(1, 3)) / wi,
										-(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
										-(u1.y*P1(2, 3) - P1(1, 3)) / wi1);

		cv::solve(A, B, X_, cv::DECOMP_SVD);

		X(0) = X_(0); 
		X(1) = X_(1); 
		X(2) = X_(2); 
		X(3) = 1.0;
	}
	return X;
}






cv::Point3d Triangulation::LinearLSTriangulation(cv::Matx34d P0, cv::Matx34d P1, cv::Point3d point3D1, cv::Point3d point3D2)
{

	cv::Matx43d A(	point3D1.x*P0(2, 0) - P0(0, 0), point3D1.x*P0(2, 1) - P0(0, 1), point3D1.x*P0(2, 2) - P0(0, 2),
				point3D1.y*P0(2, 0) - P0(1, 0), point3D1.y*P0(2, 1) - P0(1, 1), point3D1.y*P0(2, 2) - P0(1, 2),
				point3D2.x*P1(2, 0) - P1(0, 0), point3D2.x*P1(2, 1) - P1(0, 1), point3D2.x*P1(2, 2) - P1(0, 2),
				point3D2.y*P1(2, 0) - P1(1, 0), point3D2.y*P1(2, 1) - P1(1, 1), point3D2.y*P1(2, 2) - P1(1, 2));


	cv::Matx41d B(	-(point3D1.x*P0(2, 3) - P0(0, 3)),
				-(point3D1.y*P0(2, 3) - P0(1, 3)),
				-(point3D2.x*P1(2, 3) - P1(0, 3)),
				-(point3D2.y*P1(2, 3) - P1(1, 3)));


	cv::Mat_<double> X;
	cv::solve(A, B, X, cv::DECOMP_SVD);


	return cv::Point3d(X(0), X(1), X(2));

}



int Triangulation::disambiguateCameraPose(vector<cv::Point3d> points3D, cv::Mat R, cv::Mat t)
{
	int temp, temp0, temp1, temp2 = 0;

	int points_f = 0;
	int points_b = 0;

	for (int i = 0; i < points3D.size(); i++)
	{
		temp0 = R.at<double>(2, 0) * (points3D.at(i) - cv::Point3d(t)).x;
		temp1 = R.at<double>(2, 1) * (points3D.at(i) - cv::Point3d(t)).y;
		temp2 = R.at<double>(2, 2) * (points3D.at(i) - cv::Point3d(t)).z;

		temp = temp0 + temp1 + temp2;

		if (temp > 0)
		{
			points_f++;
		}
		else
		{
			points_b++;
		}

	}
	return points_f;


}






void NonLinearTriangulation(cv::Mat K, cv::Mat R1, cv::Mat t1, cv::Mat R2, cv::Mat t2, vector<cv::Point2d> image_points1, vector<cv::Point2d> image_points2, vector<cv::Point3d> points3d)
{




}




void Triangulation::writePointCloud(vector<cv::Point3d> pointCloud, int i, string objectName)
{
	ofstream outfile("pointcloud" + to_string(i) + ".ply");

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << pointCloud.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";

	for (int i = 0; i < pointCloud.size(); i++)
	{
		cv::Point3d point = pointCloud.at(i);
		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << "\n";
	}

	outfile.close();
}



void Triangulation::writeCameras(vector<cv::Point3d> cameras, int i)
{
	ofstream outfile("pointcloud" + to_string(i) + ".ply");

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << cameras.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";

	for (int i = 0; i < cameras.size(); i++)
	{
		cv::Point3d point = cameras.at(i);
		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << "\n";
	}

	outfile.close();
}



/*

void Triangulation::writePointCloudEnd(int bildNr)
{
	ofstream outfile("pointcloud_" + to_string(bildNr) + ".ply");

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << pct.points3d.size() + pct.projection_matrices.size() + (depth_vec.size()) + 1 << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n property uchar red\n" << "property uchar green\n" << "property uchar blue\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";

	for (int i = 0; i < pct.points3d.size(); i++)
	{
		Point3d point = pct.points3d.at(i);
		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << 0 << " ";
		outfile << 255 << " ";
		outfile << 0 << " ";
		outfile << "\n";
	}

	
	for (int i = 0; i < depth.rows; i++)
	{
		for (int j = 0; j < depth.cols; j++)
		{
			outfile << depth.at<Vec3f>(i, j)[0] << " ";
			outfile << depth.at<Vec3f>(i, j)[1] << " ";
			outfile << depth.at<Vec3f>(i, j)[2] << " ";
			outfile << 255 << " ";
			outfile << 255 << " ";
			outfile << 0 << " ";
			outfile << "\n";
		}
	}
	

	outfile << P1_rect.at<double>(0, 3) << " ";
	outfile << P1_rect.at<double>(1, 3) << " ";
	outfile << P1_rect.at<double>(2, 3) << " ";
	outfile << 255 << " ";
	outfile << 255 << " ";
	outfile << 255 << " ";
	outfile << "\n";


	for (int i = 0; i < depth_vec.size(); i++)
	{
			outfile << depth_vec.at(i).x << " ";
			outfile << depth_vec.at(i).y << " ";
			outfile << depth_vec.at(i).z << " ";
			outfile << 255 << " ";
			outfile << 255 << " ";
			outfile << 0 << " ";
			outfile << "\n";
	}



	for (int i = 0; i < pct.projection_matrices.size(); i++)
	{
		
		Mat P_temp_ = (Mat_<double>(4, 4) <<
			pct.projection_matrices.at(i)(0, 0), pct.projection_matrices.at(i)(0, 1), pct.projection_matrices.at(i)(0, 2), pct.projection_matrices.at(i)(0, 3),
			pct.projection_matrices.at(i)(1, 0), pct.projection_matrices.at(i)(1, 1), pct.projection_matrices.at(i)(1, 2), pct.projection_matrices.at(i)(1, 3),
			pct.projection_matrices.at(i)(2, 0), pct.projection_matrices.at(i)(2, 1), pct.projection_matrices.at(i)(2, 2), pct.projection_matrices.at(i)(2, 3),
			0, 0, 0, 1);

		Mat P_inv = P_temp_.inv();


		double x = P_inv.at<double>(0, 3);
		double y = P_inv.at<double>(1, 3);
		double z = P_inv.at<double>(2, 3);
		outfile << x << " ";
		outfile << y << " ";
		outfile << z << " ";
		outfile << i << " ";
		outfile << 0 << " ";
		outfile << 255 << " ";
		outfile << "\n";
	}



	outfile.close();
}


*/