#include "StereoMatcherSGM.h"

#include <time.h>    

/*
void onTrackbar(int, void *)
{

berechneDisparity2(image1_rect, image2_rect);


//image_disparity_div16 = image_disparity / 16;
//double minValue, maxValue;
//minMaxLoc(StereoMatcher::image_disparity_RAW_CV32F_div16, &minValue, &maxValue);
//cout << "MaxValue: " << maxValue << endl;


//image_disparity_div16.convertTo(image_disparity_ausgabe, CV_32F, 1 / maxValue);


//imshow("Disparity", StereoMatcher::image_disparity_RAW_CV32F_norm);

//berechnePointCloud();
}
*/



StereoMatcherSGM::StereoMatcherSGM()
{
	//init_disparity_method(10, 20);
}


StereoMatcherSGM::~StereoMatcherSGM()
{
	finish_disparity_method();
}



Mat StereoMatcherSGM::translateImg(Mat &img, int offsetx, int offsety)
{
	Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	warpAffine(img, img, trans_mat, img.size());
	return trans_mat;
}



void StereoMatcherSGM::rectify(Mat K, Mat image1, Mat image2, Mat R, Mat t, Mat& image1_rect, Mat& image2_rect, Mat& R1, Mat &P1, Mat &P2, Mat& Q, int minx1, int minx2, int miny1, int miny2)
{
	stereoRectify(K, Mat(), K, Mat(), image1.size(), R, t, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY);

	Mat P_temp = K.inv() * P1;
	P1 = K.inv() * P2;
	P2 = P_temp;


	cout << "R1: " << endl << R1 << endl << endl << "R2: " << endl << R2 << endl << endl << endl
		<< "P1: " << endl << P1 << endl << endl << "P2: " << endl << P2 << endl << endl;

	Mat map1x, map1y, map2x, map2y;
	initUndistortRectifyMap(K, Mat(), R1, Mat(), image1.size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(K, Mat(), R2, Mat(), image2.size(), CV_32FC1, map2x, map2y);

	remap(image1, image1_rect, map1x, map1y, INTER_LINEAR);
	remap(image2, image2_rect, map2x, map2y, INTER_LINEAR);

	//minDisp = map1x.at<float>(minx1, miny1) - map2x.at<float>(minx2, miny2); //Das muss invers sein!!!
	minDisp = (minx1 - minx2) + 100;
	cout << minDisp << endl << endl;


	translateImg(image2_rect, minDisp, 0);

	imshow("image1_rect", image1_rect);
	setWindowProperty("image1_rect", WindowPropertyFlags::WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);
	imshow("image2_rect", image2_rect);
	setWindowProperty("image2_rect", WindowPropertyFlags::WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);

	//waitKey();
}



void StereoMatcherSGM::berechneDisparity(Mat& image1_rect, Mat& image2_rect)
{
	cvtColor(image1_rect, gray_image1, CV_RGB2GRAY);
	cvtColor(image2_rect, gray_image2, CV_RGB2GRAY);

	int nParts = 8;

	for (int i = 0; i < nParts; i++)
	{
		image1_roi.push_back(gray_image1(Rect(0, i * (gray_image1.rows / nParts), gray_image1.cols, gray_image1.rows / nParts)));
		image2_roi.push_back(gray_image2(Rect(0, i * (gray_image2.rows / nParts), gray_image2.cols, gray_image2.rows / nParts)));

		
		imshow("image1_rect part" + to_string(i), image1_roi.at(i));
		imshow("image2_rect part" + to_string(i), image2_roi.at(i));
	}
	//waitKey();


	vector<Mat> image_disparity_parts;
	float elapsed_time;

	double time1 = 0.0, tstart;      // time measurment variables
	tstart = clock();              // start

	/*
	for (int i = 0; i < nParts; i++)
	{
		init_disparity_method(10, 20);
		image_disparity_parts.push_back(Mat(gray_image2.cols, gray_image2.rows / nParts, CV_8UC1));
		compute_disparity_method(image1_roi.at(i), image2_roi.at(i), &elapsed_time).copyTo(image_disparity_parts.at(i));
		finish_disparity_method();
	}
	*/

	//Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 512, 17, 2, 6, 5, 0, 15, 100, 2);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 512, 17, 2, 6, 0, 0, 0, 0, 0);
	sgbm->compute(gray_image1, gray_image2, image_disparity);
	

	time1 += clock() - tstart;     // end
	time1 = time1 / CLOCKS_PER_SEC;  // rescale to seconds
	cout << "  time = " << time1 << " sec." << endl;
	


	
	//image_disparity = image_disparity_parts.at(0);

	//for (int i = 1; i < image_disparity_parts.size(); i++)
	//{
	//	vconcat(image_disparity, image_disparity_parts.at(i), image_disparity);
	//}
	


	//image_disparity = image_disparity + 45;
	
	/*
	cv::Mat disp8;
	normalize(image_disparity, disp8, 0, 255, CV_MINMAX, CV_8U);
	cv::imshow("Disparity", disp8);
	cv::waitKey();
	*/

	
	
	image_disparity.convertTo(image_disparity_CV32F, CV_32FC3);

	
	double min, max;

	image_disparity_CV32F = image_disparity_CV32F / 16;

	cv::minMaxLoc(image_disparity_CV32F, &min, &max);
	cout << min << " " << max << endl;

	cout << image_disparity_CV32F.size() << " " << image_disparity_CV32F.type() << endl;

	


	

	//image_disparity_CV32F = disp8;

	//image_disparity_CV32F = image_disparity;

	//imshow("image_disparity", image_disparity);
	//waitKey();


}



void StereoMatcherSGM::berechnePointCloud(Mat& image1, Mat& image2, Mat K, Mat R, Mat t, Mat& R1, Mat& P1, Mat& P2, vector<Point3d>& _depth, vector<Vec3b>& _colors, int minx1, int minx2, int miny1, int miny2)
{
	rectify(K, image1, image2, R, t, image1_rect, image2_rect, R1, P1, P2, Q, minx1, minx2, miny1, miny2);

	berechneDisparity(image1_rect, image2_rect);


	//CV_Assert(image_disparity_RAW_CV32F_div16.type() == CV_8U);
	CV_Assert(!image_disparity_CV32F.empty());
	//CV_Assert(Q.type() == CV_32F && Q.cols == 4 && Q.rows == 4);

	depth = Mat::zeros(image_disparity_CV32F.size(), CV_32FC3);



	double Q03 = Q.at<double>(0, 3);
	double Q13 = Q.at<double>(1, 3);
	double Q23 = Q.at<double>(2, 3);
	double Q32 = Q.at<double>(3, 2);
	double Q33 = Q.at<double>(3, 3);


	cout << "Q: " << std::setprecision(500) << Q << endl;

	cout << "Q03: " << std::setprecision(500) << Q03 << endl;
	cout << "Q13: " << std::setprecision(500) << Q13 << endl;
	cout << "Q23: " << std::setprecision(500) << Q23 << endl;
	cout << "Q32: " << std::setprecision(500) << Q32 << endl;
	cout << "Q33: " << std::setprecision(500) << Q33 << endl << endl;



	int x = 0;

	for (int y = 0; y < image_disparity_CV32F.rows; y++)
	{
		for (x = 0; x < image_disparity_CV32F.cols; x++)
		{
			if (image_disparity_CV32F.at<float>(y, x) >= 1 && image_disparity_CV32F.at<float>(y, x) <= 512)
			{
				//cout << "y: " << y << ", x: " << x << endl;

				float pw;

				

				pw = 1.0f / ((image_disparity_CV32F.at<float>(y, x) + (minDisp)) * Q32 + Q33);

				


				Vec3f& point = depth.at<Vec3f>(y, x);


				point[0] = (static_cast<float>(x) + Q03 - (K.at<double>(0, 2) + Q.at<double>(0, 3))) * pw;
				point[1] = (static_cast<float>(y) + Q13 - (K.at<double>(1, 2) + Q.at<double>(1, 3))) * pw;

				point[2] = (Q23) * pw;


				colors.push_back(Vec3b(image1_rect.at<Vec3b>(y, x)[0], image1_rect.at<Vec3b>(y, x)[1], image1_rect.at<Vec3b>(y, x)[2]));
				_colors.push_back(Vec3b(image1_rect.at<Vec3b>(y, x)[0], image1_rect.at<Vec3b>(y, x)[1], image1_rect.at<Vec3b>(y, x)[2]));
				_depth.push_back(Point3d(point[0], point[1], point[2]));
				depth_vec.push_back(Point3d(point[0], point[1], point[2]));

				//cout << endl << "p0: " << point[0] << endl << "p1: " << point[1] << endl << "p2: " << point[2] << endl << endl;
			}
		}
	}
	//writePointCloud();
}




void StereoMatcherSGM::writePointCloud()
{
	string pfad = "Test/test.ply";
	cout << pfad;
	ofstream outfile(pfad);

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << depth_vec.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";


	for (int i = 0; i < depth_vec.size(); i++)
	{
		Point3d point = depth_vec.at(i);

		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << "\n";
	}



	outfile.close();
}

