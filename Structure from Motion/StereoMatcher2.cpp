#include "StereoMatcher2.h"
/*


cv::Ptr<cv::StereoBM> bm;
int minDisparity;
int numDisparities;
int blockSize;

cv::Mat image1_rect;
cv::Mat image2_rect;


cv::Mat image_disparity_RAW;
cv::Mat image_disparity_RAW_CV32F;
cv::Mat image_disparity_RAW_CV32F_norm;
cv::Mat image_disparity_RAW_CV32F_div16;
cv::Mat depth;




static void berechneDisparity2(static cv::Mat image1_rect, static cv::Mat image2_rect)
{

	bm->setMinDisparity(minDisparity);
	bm->setNumDisparities(numDisparities + (16 - (numDisparities % 16)));
	if (blockSize % 2 == 1)
		bm->setBlockSize(blockSize);


	cvtColor(image1_rect, image1_rect, CV_BGR2GRAY);
	cvtColor(image2_rect, image2_rect, CV_BGR2GRAY);

	//cv::cuda::GpuMat image1_rect_Gpu, image2_rect_Gpu;

	//image1_rect_Gpu.upload(image1_rect);
	//image2_rect_Gpu.upload(image2_rect);

	//cv::cuda::GpuMat image_disparity_RAW_Gpu(image2_rect_Gpu.size(), CV_32FC3);


	bm->compute(image1_rect, image2_rect, image_disparity_RAW);


	//image_disparity_RAW_Gpu.download(image_disparity_RAW);

	cout << image_disparity_RAW.type() << endl;

	normalize(image_disparity_RAW, image_disparity_RAW_CV32F_norm, 0, 1, cv::NORM_MINMAX);
	cv::imshow("Disparity", image_disparity_RAW);
	cv::waitKey();

	cout << image_disparity_RAW.size() << endl;



	//image_disparity_RAW.convertTo(image_disparity_RAW_CV32F, CV_32FC3);
	image_disparity_RAW.convertTo(image_disparity_RAW_CV32F_div16, CV_32FC3, 1/ 16.);

	//image_disparity_RAW_CV32F_div16 = image_disparity_RAW;
	//cout << image_disparity_RAW_CV32F_div16;
	cout << "Type: " << image_disparity_RAW_CV32F_div16.type() << endl;



	double min, max;
	cv::minMaxLoc(image_disparity_RAW_CV32F_div16, &min, &max);
	cout << "Min: " << min << ", Max: " << max << endl;



	//cout << image_disparity_RAW << endl;

	//cout << image_disparity_RAW_CV32F_norm.size() << endl;


	//image_disparity_RAW_CV32F_div16 = image_disparity_RAW_CV32F / 16;

	//cout << image_disparity_RAW_CV32F_div16.size() << endl;



}













void onTrackbar(int, void *)
{

	berechneDisparity2(image1_rect, image2_rect);


	//image_disparity_div16 = image_disparity / 16;
	//double minValue, maxValue;
	//minMaxLoc(StereoMatcher::image_disparity_RAW_CV32F_div16, &minValue, &maxValue);
	//std::cout << "MaxValue: " << maxValue << endl;


	//image_disparity_div16.convertTo(image_disparity_ausgabe, CV_32F, 1 / maxValue);


	//cv::imshow("Disparity", StereoMatcher::image_disparity_RAW_CV32F_norm);

	//berechnePointCloud();
}




StereoMatcher2::StereoMatcher2() 
{
	

	//sgbm = cv::StereoSGBM::create();
	//minDisparity = 350;
	minDisparity = -1700;
	//numDisparities = 35*16;
	numDisparities = 2 * 16;
	blockSize = 5;

	bm = cv::StereoBM::create(numDisparities, blockSize);

	namedWindow("Disparity", cv::WINDOW_NORMAL);

	cv::createTrackbar("minDisparity", "Disparity", &minDisparity, 500, onTrackbar);
	cv::createTrackbar("numDisparities", "Disparity", &numDisparities, 1600, onTrackbar);
	cv::createTrackbar("SADWindowSize", "Disparity", &blockSize, 200, onTrackbar);

}





void StereoMatcher2::rectify(cv::Mat K, cv::Mat image1, cv::Mat image2, cv::Mat R, cv::Mat t, cv::Mat &R1, cv::Mat &P2, vector<cv::Point3d> &depth_, vector<cv::Vec3b> &colors_, int minDisp) {
	
	cv::stereoRectify(K, cv::Mat(), K, cv::Mat(), image1.size(), R, t, R1, R2, P1, P2, Q);

	//cv::normalize(P1, P1, 1);
	//cv::normalize(P2, P2, 1);
	//cv::normalize(Q, Q, 1);
	cout << "xkeiojkio: " << endl << endl << R1 << endl << endl << R2 << endl << endl << P1 << endl << endl << P2 << endl << endl;

	cv::Mat map1x, map1y, map2x, map2y;
	cv::initUndistortRectifyMap(K, cv::Mat(), R1, cv::Mat(), image1.size(), CV_32FC1, map1x, map1y);
	cv::initUndistortRectifyMap(K, cv::Mat(), R2, cv::Mat(), image1.size(), CV_32FC1, map2x, map2y);

	cv::remap(image1, image1_rect, map1x, map1y, cv::INTER_LINEAR);
	cv::remap(image2, image2_rect, map2x, map2y, cv::INTER_LINEAR);


	cv::imshow("image1_rect", image1_rect);
	cv::setWindowProperty("image1_rect", cv::WindowPropertyFlags::WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);
	cv::imshow("image2_rect", image2_rect);
	cv::setWindowProperty("image2_rect", cv::WindowPropertyFlags::WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);

	cv::waitKey();

	cout << t << endl;





	berechneDisparity2(image1_rect, image2_rect);
	berechnePointCloud(Q, depth_, colors_);
	//writePointCloud();
}













void StereoMatcher2::berechnePointCloud(cv::Mat Q, vector<cv::Point3d>& depth_, vector<cv::Vec3b>& colors_)
{
	//CV_Assert(image_disparity_RAW_CV32F_div16.type() == CV_8U);
	CV_Assert(!image_disparity_RAW_CV32F_div16.empty());
	//CV_Assert(Q.type() == CV_32F && Q.cols == 4 && Q.rows == 4);

	depth = cv::Mat::zeros(image_disparity_RAW_CV32F_div16.size(), CV_32FC3);



	double Q03 = Q.at<double>(0, 3);
	double Q13 = Q.at<double>(1, 3);
	double Q23 = Q.at<double>(2, 3);
	double Q32 = Q.at<double>(3, 2);
	double Q33 = Q.at<double>(3, 3);


	cout << "Q: " << Q << endl;

	cout << "Q03: " << Q03 << endl;
	cout << "Q13: " << Q13 << endl;
	cout << "Q23: " << Q23 << endl;
	cout << "Q32: " << Q32 << endl;
	cout << "Q33: " << Q33 << endl << endl;





	for (int y = 0; y < image_disparity_RAW_CV32F_div16.rows; y++)
	{
		for (int x = 0; x < image_disparity_RAW_CV32F_div16.cols; x++)
		{
			if (image_disparity_RAW_CV32F_div16.at<float>(y, x) > minDisparity) {
				//cout << "y: " << y << ", x: " << x << endl;

				const float pw = 1.0f / (image_disparity_RAW_CV32F_div16.at<float>(y, x) * Q32 + Q33);

				cv::Vec3f& point = depth.at<cv::Vec3f>(y, x);

				point[0] = (static_cast<float>(x) + Q03) * pw;
				point[1] = (static_cast<float>(y) + Q13) * pw;
				point[2] = Q23 * pw;

				colors.push_back(cv::Point3d(image1_rect.at<cv::Vec3b>(y, x)));
				colors_.push_back(cv::Vec3b(image1_rect.at<cv::Vec3b>(y, x)));
				depth_.push_back(cv::Point3d(point[0], point[1], point[2]));
				depth_vec.push_back(cv::Point3d(point[0], point[1], point[2]));

				//cout << endl << "p0: " << point[0] << endl << "p1: " << point[1] << endl << "p2: " << point[2] << endl << endl;
			}
		}
	}


}




void StereoMatcher2::writePointCloud()
{
	string pfad = "Test/test.ply";
	cout << pfad;
	ofstream outfile(pfad);

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << depth_vec.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n property uchar red\n" << "property uchar green\n" << "property uchar blue\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";


	for (int i = 0; i < depth_vec.size(); i++)
	{
		cv::Point3d point = depth_vec.at(i);

		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << colors.at(i).x << " ";
		outfile << colors.at(i).y << " ";
		outfile << colors.at(i).z << " ";
		outfile << "\n";
	}



	outfile.close();
}

*/