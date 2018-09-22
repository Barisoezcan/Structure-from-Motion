#include "Bild.h"




void initUndistortRectifyMap_fisheye(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _matR, cv::InputArray _newCameraMatrix, cv::Size size, int m1type, cv::OutputArray _map1, cv::OutputArray _map2)
{
	cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
	cv::Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();

	if (m1type <= 0)
		m1type = CV_16SC2;
	CV_Assert(m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2);
	_map1.create(size, m1type);
	cv::Mat map1 = _map1.getMat(), map2;
	if (m1type != CV_32FC2)
	{
		_map2.create(size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1);
		map2 = _map2.getMat();
	}
	else
		_map2.release();

	cv::Mat_<double> R = cv::Mat_<double>::eye(3, 3);
	cv::Mat_<double> A = cv::Mat_<double>(cameraMatrix), Ar;

	if (!newCameraMatrix.empty())
		Ar = cv::Mat_<double>(newCameraMatrix);
	else
		Ar = getDefaultNewCameraMatrix(A, size, true);

	if (!matR.empty())
		R = cv::Mat_<double>(matR);

	if (!distCoeffs.empty())
		distCoeffs = cv::Mat_<double>(distCoeffs);
	else
	{
		distCoeffs.create(14, 1, CV_64F);
		distCoeffs = 0.;
	}

	CV_Assert(A.size() == cv::Size(3, 3) && A.size() == R.size());
	CV_Assert(Ar.size() == cv::Size(3, 3) || Ar.size() == cv::Size(4, 3));
	cv::Mat_<double> iR = (Ar.colRange(0, 3)*R).inv(cv::DECOMP_LU);
	const double* ir = &iR(0, 0);

	double u0 = A(0, 2), v0 = A(1, 2);
	double fx = A(0, 0), fy = A(1, 1);

	CV_Assert(distCoeffs.size() == cv::Size(1, 4) || distCoeffs.size() == cv::Size(4, 1) ||
		distCoeffs.size() == cv::Size(1, 5) || distCoeffs.size() == cv::Size(5, 1) ||
		distCoeffs.size() == cv::Size(1, 8) || distCoeffs.size() == cv::Size(8, 1) ||
		distCoeffs.size() == cv::Size(1, 12) || distCoeffs.size() == cv::Size(12, 1) ||
		distCoeffs.size() == cv::Size(1, 14) || distCoeffs.size() == cv::Size(14, 1));

	if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
		distCoeffs = distCoeffs.t();

	const double* const distPtr = distCoeffs.ptr<double>();
	double k1 = distPtr[0];
	double k2 = distPtr[1];
	double p1 = distPtr[2];
	double p2 = distPtr[3];
	double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? distPtr[4] : 0.;
	double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[5] : 0.;
	double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[6] : 0.;
	double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? distPtr[7] : 0.;
	double s1 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[8] : 0.;
	double s2 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[9] : 0.;
	double s3 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[10] : 0.;
	double s4 = distCoeffs.cols + distCoeffs.rows - 1 >= 12 ? distPtr[11] : 0.;
	double tauX = distCoeffs.cols + distCoeffs.rows - 1 >= 14 ? distPtr[12] : 0.;
	double tauY = distCoeffs.cols + distCoeffs.rows - 1 >= 14 ? distPtr[13] : 0.;

	// Matrix for trapezoidal distortion of tilted image sensor
	cv::Matx33d matTilt = cv::Matx33d::eye();
	cv::detail::computeTiltProjectionMatrix(tauX, tauY, &matTilt);

	for (int i = 0; i < size.height; i++)
	{
		float* m1f = map1.ptr<float>(i);
		float* m2f = map2.empty() ? 0 : map2.ptr<float>(i);
		short* m1 = (short*)m1f;
		ushort* m2 = (ushort*)m2f;
		double _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];

		for (int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6])
		{
			double w = 1. / _w, x_temp = _x*w, y_temp = _y*w;
			double x_temp2 = x_temp*x_temp, y_temp2 = y_temp*y_temp;
			//double r2 = x2 + y2, _2xy = 2*x*y;
			double r_temp = sqrt(x_temp2 + y_temp2);
			double x = x_temp * atan(r_temp) / r_temp;
			double y = y_temp * atan(r_temp) / r_temp;
			double r = sqrt(x*x + y*y);
			double r2 = r*r;
			double kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4)*r2);
			double xd = (x*kr + p1 * 2 * x*y + p2*(r2 + 2 * x*x) + s1*r2 + s2*r2*r2);
			double yd = (y*kr + p1*(r2 + 2 * y*y) + p2 * 2 * x*y + s3*r2 + s4*r2*r2);
			cv::Vec3d vecTilt = matTilt*cv::Vec3d(xd, yd, 1);
			double invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
			double u = fx*invProj*vecTilt(0) + u0;
			double v = fy*invProj*vecTilt(1) + v0;
			if (m1type == CV_16SC2)
			{
				int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
				int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
				m1[j * 2] = (short)(iu >> cv::INTER_BITS);
				m1[j * 2 + 1] = (short)(iv >> cv::INTER_BITS);
				m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE - 1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE - 1)));
			}
			else if (m1type == CV_32FC1)
			{
				m1f[j] = (float)u;
				m2f[j] = (float)v;
			}
			else
			{
				m1f[j * 2] = (float)u;
				m1f[j * 2 + 1] = (float)v;
			}
		}
	}
}



















void undistort_fisheye(cv::InputArray _src, cv::OutputArray _dst, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _newCameraMatrix)
{

	cv::Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
	cv::Mat distCoeffs = _distCoeffs.getMat(), newCameraMatrix = _newCameraMatrix.getMat();

	_dst.create(src.size(), src.type());
	cv::Mat dst = _dst.getMat();

	CV_Assert(dst.data != src.data);

	int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
	cv::Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);

	cv::Mat_<double> A, Ar, I = cv::Mat_<double>::eye(3, 3);

	cameraMatrix.convertTo(A, CV_64F);
	if (!distCoeffs.empty())
		distCoeffs = cv::Mat_<double>(distCoeffs);
	else
	{
		distCoeffs.create(5, 1, CV_64F);
		distCoeffs = 0.;
	}

	if (!newCameraMatrix.empty())
		newCameraMatrix.convertTo(Ar, CV_64F);
	else
		A.copyTo(Ar);

	double v0 = Ar(1, 2);
	for (int y = 0; y < src.rows; y += stripe_size0)
	{
		int stripe_size = std::min(stripe_size0, src.rows - y);
		Ar(1, 2) = v0 - y;
		cv::Mat map1_part = map1.rowRange(0, stripe_size),
			map2_part = map2.rowRange(0, stripe_size),
			dst_part = dst.rowRange(y, y + stripe_size);

		initUndistortRectifyMap_fisheye(A, distCoeffs, I, Ar, cv::Size(src.cols, stripe_size),
			map1_part.type(), map1_part, map2_part);
		remap(src, dst_part, map1_part, map2_part, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	}
}


void rot90(cv::Mat &matImage, int rotflag) {
	//1=CW, 2=CCW, 3=180
	if (rotflag == 1) {
		transpose(matImage, matImage);
		flip(matImage, matImage, 1); //transpose+flip(1)=CW
	}
	else if (rotflag == 2) {
		transpose(matImage, matImage);
		flip(matImage, matImage, 0); //transpose+flip(0)=CCW     
	}
	else if (rotflag == 3) {
		flip(matImage, matImage, -1);    //flip(-1)=180          
	}
	else if (rotflag != 0) { //if not 0,1,2,3:
		cout << "Unknown rotation flag(" << rotflag << ")" << endl;
	}
}




Bild::Bild(int _Bild_nr, cv::Mat _image, cv::Mat _K, cv::Mat _distCoeffs)
{
	Bild_nr = _Bild_nr;
	image_distorted = _image;

	_K.copyTo(K);
	_distCoeffs.copyTo(distCoeffs);





	//undistort_fisheye(image_distorted, image, K, distCoeffs, Mat());
	undistort(image_distorted, image, K, distCoeffs);
	//image_distorted.copyTo(image);
	//image.copyTo(image_gray);
	cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);


	imageGPU.upload(image_gray);
	
	//detector = cv::xfeatures2d::SURF::create(minHessian);
	//detector = SURF::create(minHessian);
	//detector2 = FastFeatureDetector::create();
	berechneKeypoints();
	berechneDescriptors();
	berechneFeaturePoints();

	//cv::Mat image_key;
	//cv::drawKeypoints(image, keypoints, image_key);
	//cv::imshow("Bild mit Feature Points", image_key);
	//cv::waitKey();
	cout << "#Descriptors: " << descriptors.rows << endl;

	//InitSiftData(siftData, 25000, true, true);
}


Bild::Bild()
{
}


void Bild::berechneKeypoints()
{
	cout << "Testtt" << endl;
	//surf(imageGPU, cv::cuda::GpuMat(), keypointsGPU, descriptorsGPU);
	//surf.downloadKeypoints(keypointsGPU, keypoints);
	//surf.downloadDescriptors(descriptorsGPU, descriptors);

	//surf.releaseMemory();

	keypoints.clear();



	//CudaImage img;
	//img.Allocate(image.cols, image.rows, image.cols, false, NULL, (float*)image.data);
	/* Download image from host to device */
	//img.Download();

	int numOctaves = 5;    /* Number of octaves in Gaussian pyramid */
	float initBlur = 1.0f; /* Amount of initial Gaussian blurring in standard deviations */
	float thresh = 3.5f;   /* Threshold on difference of Gaussians for feature pruning */
	float minScale = 0.0f; /* Minimum acceptable scale to remove fine-scale features */
	bool upScale = false;  /* Whether to upscale image before extraction */
						   /* Extract SIFT features */
	//ExtractSift(siftData, img, numOctaves, initBlur, thresh, minScale, upScale);


	/*
	for (int i = 0; i < siftData.numPts; i++)
	{
		cv::Point2f p(siftData.d_data[i].xpos, siftData.d_data[i].ypos);
		keypoints.push_back(cv::KeyPoint(p, 1.f));

	}
	*/
	cv::Ptr<SURF> detector = SURF::create(minHessian);


	detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);


	//keypointsGPU.upload(keypoints);
	//descriptorsGPU.upload(descriptors);


	//surf(imageGPU, cv::cuda::GpuMat(), keypointsGPU, descriptorsGPU);

	//surf.downloadKeypoints(keypointsGPU, keypoints);
	//surf.downloadDescriptors(descriptorsGPU, descriptors);


	//keypointsGPU.release();
	//descriptorsGPU.release();
		
	/* Free space allocated from SIFT features */
	//FreeSiftData(siftData);




	//detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);


}


void Bild::berechneDescriptors(void)
{
	//detector2->compute(image, keypoints, descriptors);
	//detector->compute(image, keypoints, descriptors);
}


void Bild::berechneFeaturePoints(void)
{
	keypointsGPU.upload(keypoints);
	for (int i = 0; i < keypoints.size(); i++)
	{
		feature_points.push_back(keypoints.at(i).pt);
	}
}