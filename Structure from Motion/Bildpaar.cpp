#include "Bildpaar.h"


Bildpaar::Bildpaar(){}


Bildpaar::Bildpaar(Bild* _Bild1, Bild* _Bild2) : isPaar(false)
{
	ratio = 0.8f;
	bild1_nr = (*_Bild1).Bild_nr;
	bild2_nr = (*_Bild2).Bild_nr;
	Bild1 = *_Bild1;
	Bild2 = *_Bild2;

	berechneGuteMatches(Bild1.descriptors, Bild2.descriptors, guteMatches);
	berechnePunktpaare(guteMatches, Bild1.keypoints, Bild2.keypoints, image_points1, image_points2);

	if (isPaar)
	{
		//cv::Mat image_matches;
		//cv::drawMatches(Bild1.image, Bild1.keypoints, Bild2.image, Bild2.keypoints, guteMatches, image_matches);
		//cv::imshow("Image_Matches", image_matches);
		//cv::imwrite("Image_Matches.png", image_matches);
		//cv::waitKey();

		F = berechneF(image_points1, image_points2, guteMatches);
		E = findEssentialMat(image_points1, image_points2, Bild1.K);


		cv::Mat_<double> R1, R2, t1, t2;
		//int inlier_passing = recoverPose(E, image_points1, image_points2, _Bild1.K, R, t);



		DecomposeEtoRandT(E, R, t);

		cout << "R: " << endl << R << endl << endl << "t: " << endl << t << endl << endl;

	}
}



void Bildpaar::berechneGuteMatches(cv::Mat descriptors1, cv::Mat descriptors2, vector<cv::DMatch>& guteMatches)
{

	//Datenstrukturen deklarieren
	vector<vector<cv::DMatch>> matches1zu2_k;
	vector<vector<cv::DMatch>> matches2zu1_k;

	vector<cv::DMatch> unique_matches1zu2;
	vector<cv::DMatch> unique_matches2zu1;

	vector<cv::DMatch> symmMatches;

	vector<cv::DMatch> filteredMatches;

	vector<cv::DMatch> matches;


	//First Matching
	cv::cuda::GpuMat descriptors1GPU, descriptors2GPU;
	descriptors1GPU.upload(descriptors1);
	descriptors2GPU.upload(descriptors2);

	//cv::BFMatcher matcher1;

	cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher();
	matcher->knnMatch(descriptors1GPU, descriptors2GPU, matches1zu2_k, 2);
	matcher->knnMatch(descriptors2GPU, descriptors1GPU, matches2zu1_k, 2);
	//matcher1.knnMatch(descriptors1, descriptors2, matches1zu2_k, 2);
	//matcher1.knnMatch(descriptors2, descriptors1, matches2zu1_k, 2);

	//Ptr<DescriptorMatcher> matcher2 = DescriptorMatcher::create(1);


	//matcher.knnMatch(Bild1.descriptors, Bild2.descriptors, matches1zu2_k, 2);
	//matcher.knnMatch(Bild2.descriptors, Bild1.descriptors, matches2zu1_k, 2);


	//Weiteres Filtern der Matches
	ratioTest(matches1zu2_k, unique_matches1zu2);
	ratioTest(matches2zu1_k, unique_matches2zu1);

	symmTest(unique_matches1zu2, unique_matches2zu1, symmMatches);


	berechnePunktpaare(symmMatches, Bild1.keypoints, Bild2.keypoints, image_points1, image_points2);



	if (image_points1.size() > 100)
	{
		ransacFilter(image_points1, image_points2, symmMatches, filteredMatches);
	}

	guteMatches = filteredMatches;


	if (guteMatches.size() >= 100)
	{
		isPaar = true;
	}

}


void Bildpaar::ratioTest(vector<vector<cv::DMatch>> matches, vector<cv::DMatch> &unique_matches)
{
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches.at(i).at(0).distance / matches.at(i).at(1).distance < ratio)
		{
			unique_matches.push_back(matches.at(i).at(0));
		}
	}
}


void Bildpaar::symmTest(vector<cv::DMatch> matches1zu2, vector<cv::DMatch> matches2zu1, vector<cv::DMatch> &symmMatches)
{
	for (int i = 0; i < matches1zu2.size(); i++)
	{
		for (int j = 0; j < matches2zu1.size(); j++)
		{
			if ((matches1zu2.at(i).trainIdx == matches2zu1.at(j).queryIdx) && (matches1zu2.at(i).queryIdx == matches2zu1.at(j).trainIdx))
			{
				symmMatches.push_back(cv::DMatch(matches1zu2.at(i).queryIdx, matches1zu2.at(i).trainIdx, matches1zu2.at(i).distance));
				break;
			}
		}
	}
}


void Bildpaar::ransacFilter(vector<cv::Point2d> image_points1, vector<cv::Point2d> image_points2, vector<cv::DMatch> symmMatches, vector<cv::DMatch> &filteredMatches)
{	
	vector<uchar> inliers;
	
	findFundamentalMat(image_points1, image_points2, CV_FM_RANSAC, 1.0, 0.999, inliers);


	for (int i = 0; i < symmMatches.size(); i++)
	{


	}


	for (int i = 0; i < inliers.size(); i++)
	{
		if (inliers.at(i) == true)
		{
			filteredMatches.push_back(symmMatches.at(i));
		}
	}
	return;
}


void Bildpaar::berechnePunktpaare(vector<cv::DMatch> guteMatches, vector<cv::KeyPoint> keypoints1, vector<cv::KeyPoint> keypoints2, vector<cv::Point2d>& image_points1, vector<cv::Point2d>& image_points2)
{
	image_points1.clear();
	image_points2.clear();

	for (int i = 0; i < guteMatches.size(); i++)
	{
		image_points1.push_back(keypoints1.at(guteMatches.at(i).queryIdx).pt);
		image_points2.push_back(keypoints2.at(guteMatches.at(i).trainIdx).pt);
	}

}



cv::Mat Bildpaar::berechneF(vector<cv::Point2d> image_points1, vector<cv::Point2d> image_points2, vector<cv::DMatch> guteMatches)
{
	F = findFundamentalMat(image_points1, image_points2, CV_FM_LMEDS);

	return F;
}




void TakeSVDOfE(cv::Mat& E, cv::Mat& svd_u, cv::Mat& svd_vt, cv::Mat& svd_w)
{
#if 1
	//Using OpenCV's SVD
	cv::SVD svd(E, cv::SVD::MODIFY_A);
	svd_u = svd.u;
	svd_vt = svd.vt;
	svd_w = svd.w;
#else
	//Using Eigen's SVD
	cout << "Eigen3 SVD..\n";
	Eigen::Matrix3f  e = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >((double*)E.data).cast<float>();
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(e, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXf Esvd_u = svd.matrixU();
	Eigen::MatrixXf Esvd_v = svd.matrixV();
	svd_u = (Mat_<double>(3, 3) << Esvd_u(0, 0), Esvd_u(0, 1), Esvd_u(0, 2),
		Esvd_u(1, 0), Esvd_u(1, 1), Esvd_u(1, 2),
		Esvd_u(2, 0), Esvd_u(2, 1), Esvd_u(2, 2));
	Mat_<double> svd_v = (Mat_<double>(3, 3) << Esvd_v(0, 0), Esvd_v(0, 1), Esvd_v(0, 2),
		Esvd_v(1, 0), Esvd_v(1, 1), Esvd_v(1, 2),
		Esvd_v(2, 0), Esvd_v(2, 1), Esvd_v(2, 2));
	svd_vt = svd_v.t();
	svd_w = (Mat_<double>(1, 3) << svd.singularValues()[0], svd.singularValues()[1], svd.singularValues()[2]);
#endif

	cout << "----------------------- SVD ------------------------\n";
	cout << "U:\n" << svd_u << "\nW:\n" << svd_w << "\nVt:\n" << svd_vt << endl;
	cout << "----------------------------------------------------\n";
}



bool Bildpaar::DecomposeEtoRandT(cv::Mat& E, cv::Mat& R, cv::Mat& t)
{
	//Using HZ E decomposition
	cv::Mat svd_u, svd_vt, svd_w;
	TakeSVDOfE(E, svd_u, svd_vt, svd_w);

	//check if first and second singular values are the same (as they should be)
	double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
	if (singular_values_ratio>1.0) singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7)
	{
		cout << "singular values are too far apart\n";
		return false;
	}

	cv::Matx33d W(0, -1, 0,	//HZ 9.13
		1, 0, 0,
		0, 0, 1);
	cv::Matx33d Wt(0, 1, 0,
		-1, 0, 0,
		0, 0, 1);
	cv::Mat R1 = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
	cv::Mat R2 = svd_u * cv::Mat(Wt) * svd_vt; //HZ 9.19
	cv::Mat t1 = svd_u.col(2); //u3
	cv::Mat t2 = -svd_u.col(2); //u3

	cout << "R1: " << endl << R1 << endl << endl << "R2: " << endl << R2 << endl << endl;

	if ((R1.at<double>(0, 0) > 0.9 && R1.at<double>(1, 1) > 0.9 && R1.at<double>(2, 2) > 0.9) 
		||
		(R1.at<double>(0, 0) < -0.9 && R1.at<double>(1, 1) < -0.9 && R1.at<double>(2, 2) < -0.9))
	{
		R = -R1;
	}
	else if ((R2.at<double>(0, 0) > 0.9 && R2.at<double>(1, 1) > 0.9 && R2.at<double>(2, 2) > 0.9)
		||
		(R2.at<double>(0, 0) < -0.9 && R2.at<double>(1, 1) < -0.9 && R2.at<double>(2, 2) < -0.9))
	{
		R = R2;
	}
	else
	{
		cout << "ERRROOOOOOOOOOOOOOOOOOOOOOORRRR!!! coudn't find a good rotation matrix!!" << endl;
	}


	if (t1.at<double>(0) < -0.9/* && t1.at<double>(1) < 0.1 && t1.at<double>(2) < 0.1*/)
	{
		t = t1;
	}
	else if (t2.at<double>(0) <- 0.9 /*&& t2.at<double>(1) < 0.1 && t2.at<double>(2) < 0.1*/)
	{
		t = t2;
	}
	else
	{
		cout << "ERRROOOOOOOOOOOOOOOOOOOOOOORRRR!!! coudn't find a good translation vector!!" << endl;
	}
	cout << "t1: " << endl << t1 << endl << endl << "t2: " << endl << t2 << endl << endl;




	return true;
}