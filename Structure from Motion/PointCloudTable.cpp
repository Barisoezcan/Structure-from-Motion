#include "PointCloudTable.h"

#include <iomanip>

/*
void PointCloudTable::push_points3D_2D_init(int image_nr, vector<Point3d> _points3D, vector<Point2d> _points2D)
{
	measured_3d obs;

	cout << _points2D.size() << endl;

	for (int i = 0; i < _points2D.size(); i++)
	{
		obs.points3d_nr.push_back(i);
		obs.points2d.push_back(_points2D.at(i));

		points3d.push_back(_points3D.at(i));
	}

	observations.push_back(obs);
}
*/




PointCloudTable::PointCloudTable(int _bild_anzahl)
{
	bild_anzahl = _bild_anzahl;
	//denseClouds.resize(_bild_anzahl);
	//denseCloudsColor.resize(_bild_anzahl);

};




void PointCloudTable::push_measurement_init(int image_nr1, int image_nr2, cv::Mat image1, cv::Mat image2, vector<cv::Point3d> _new_points3D, vector<cv::KeyPoint> _KP_firstCam, vector<cv::KeyPoint> _KP_secondCam, vector<cv::DMatch> matches, map<pair<int, int>, Bildpaar> bildpaare)
{
	//observations_anzahl = 0;

	for (int i = 0; i < _new_points3D.size(); i++)
	{
		bool found_in_other_view = false;


		for (int j = 0; j < bild_anzahl; j++)
		{
			if (j != image_nr1)
			{
				vector<cv::DMatch> submatches = bildpaare[make_pair(j, image_nr2)].guteMatches;

				for (int ii = 0; ii < submatches.size(); ii++)
				{
					if (submatches[ii].trainIdx == matches.at(i).trainIdx && !found_in_other_view)
					{
						for (int pt3d = 0; pt3d < points3d_measured.size(); pt3d++)
						{
							if (points3d_measured.at(pt3d).imgs_pnts.at(j) == submatches[ii].queryIdx)
							{
								if (points3d_measured.at(pt3d).imgs_pnts.at(image_nr1) == -1 && points3d_measured.at(pt3d).imgs_pnts.at(image_nr2) == -1)
									observations_anzahl += 2;
								else if (points3d_measured.at(pt3d).imgs_pnts.at(image_nr1) == -1 ^ points3d_measured.at(pt3d).imgs_pnts.at(image_nr2) == -1)
									observations_anzahl++;

								points3d_measured.at(pt3d).imgs_pnts.at(image_nr2) = matches.at(i).trainIdx;
								points3d_measured.at(pt3d).imgs_pnts.at(image_nr1) = matches.at(i).queryIdx;

								found_in_other_view = true;
								break;
							}

						}
						
					}
				}
			}
		}

		if (!found_in_other_view)
		{
			measured_3d temp2;

			temp2.XYZ = _new_points3D.at(i);

			temp2.RGB[2] = image1.at<cv::Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[0];
			temp2.RGB[1] = image1.at<cv::Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[1];
			temp2.RGB[0] = image1.at<cv::Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[2];


			temp2.imgs_pnts.resize(bild_anzahl, -1);
			temp2.imgs_pnts.at(image_nr1) = matches.at(i).queryIdx;
			temp2.imgs_pnts.at(image_nr2) = matches.at(i).trainIdx;

			points3d_measured.push_back(temp2);

			observations_anzahl += 2;

		}
		
	}
	//points3d_measured.push_back(img_pnt1);
	//imgs_pnts.push_back(img_pnt1);


	cout << "observations_anzahl: " << observations_anzahl << endl;
}


/*
void PointCloudTable::push_measurement(int image_nr, Mat image, vector<Point3d> _new_points3D, vector<KeyPoint> _KP_firstCam, vector<DMatch> matches)
{
	vector<int> img_pnt1;
	vector<int> img_pnt2;

	for (int i = 0; i < _new_points3D.size(); i++)
	{
		measured_3d temp;

		temp.imgs_pnts.push_back(matches.at(i).queryIdx);
		temp.imgs_pnts.push_back(matches.at(i).trainIdx);

		temp.RGB[2] = image1.at<Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[0];
		temp.RGB[1] = image1.at<Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[1];
		temp.RGB[0] = image1.at<Vec3b>(_KP_firstCam.at(matches.at(i).queryIdx).pt.y, _KP_firstCam.at(matches.at(i).queryIdx).pt.x)[2];
		temp.XYZ = _new_points3D.at(i);


		points3d_measured.push_back(temp);
	}
	//points3d_measured.push_back(img_pnt1);
	//imgs_pnts.push_back(img_pnt1);



}
*/








void PointCloudTable::find_point3D_init(int bild_nr, vector<cv::KeyPoint> _kp2D_toFind_new, map<pair<int,int>, Bildpaar> bildpaare, vector<cv::Point2d>& points2d_found, vector<cv::Point3d>& points3d_found)
{
	points2d_found.clear();
	points3d_found.clear();


	for (int bp_i = 0; bp_i < bild_anzahl; bp_i++)
	{
		vector<cv::DMatch> bp_matches = bildpaare[make_pair(bp_i, bild_nr)].guteMatches;


		for (int i = 0; i < bp_matches.size(); i++)
		{
			for (int j = 0; j < points3d_measured.size(); j++)
			{
				if (bp_matches.at(i).queryIdx == points3d_measured.at(j).imgs_pnts.at(bp_i))
				{
					cv::Point2d point2d_found(_kp2D_toFind_new.at(bp_matches.at(i).trainIdx).pt);
					//Point2d point2d_found(points3d_measured.at(j).imgs_pnts.at(bp_i));
					points2d_found.push_back(point2d_found);
					points3d_found.push_back(points3d_measured.at(j).XYZ);
				}
			}
		}
	}
}



/*
void PointCloudTable::find_point3D(vector<Point2d> _points2D_toFind_old, vector<Point2d> _points2D_toFind_new, vector<Point2d>& points2d_found, vector<Point3d>& points3d_found)
{
	//mask.clear();
	bool found = false;

	for (int i = 0; i < _points2D_toFind_old.size(); i++)
	{
		found = false;
		for (int j = 0; j < points3d_measured.size(); j++)
		{
			for (int k = 0; k < points3d_measured.at(j).observations.size(); k++)
				if (_points2D_toFind_old.at(i) == points3d_measured.at(j).observations.at(k).xy)
				{
					points2d_found.push_back(_points2D_toFind_new.at(i));
					points3d_found.push_back(points3d_measured.at(j).XYZ);
					//mask.push_back(observations.back().points3d_nr.at(j));
					found = true;
					break;
				}
		}
		//if (!found)
		//{
		//	mask.push_back(-1);
		//}
	}
}
*/

/*
void PointCloudTable::find_point3D(vector<Point2d> _points2D_toFind_old, vector<Point2d> _points2D_toFind_new, vector<Point2d>& points2d_found, vector<Point3d>& points3d_found, Mat& mask)
{
	mask = Mat::zeros(_points2D_toFind_old.size(), 1, CV_8UC1);

	for (int i = 0; i < _points2D_toFind_old.size(); i++)
	{
		for (int j = 0; j < points2d.size(); j++)
		{
			if (_points2D_toFind_old.at(i) == points2d.at(j).point2d)
			{
				points2d_found.push_back(_points2D_toFind_new.at(i));
				points3d_found.push_back(points3d.at(points2d.at(j).point3d_nr));
				mask.at<int>(j, 0) = points2d.at(j).point3d_nr;
				break;
			}
		}
	}
}
*/




void PointCloudTable::writeHeader(int num_cameras, int num_points, int num_observations)
{
	ofstream outfile("points0.txt");

	outfile << "NVM_3 \n\n" << num_observations << endl;
	outfile.close();
}

/*
void PointCloudTable::writePointCloudData(int i)
{
	ofstream outfile("points0.txt");

	outfile << i << 


}
*/


void PointCloudTable::writeCameras(vector<string> camera_names, vector<cv::Mat> Ks)
{
	ofstream outfile("points0.txt");

	outfile << camera_names.size();

	for (int i = 0; i < camera_names.size(); i++)
	{
		outfile << camera_names.at(i) << "\t" << Ks.at(i) << endl;
	}
	outfile << endl;

	outfile.close();
}


void PointCloudTable::writeAllDenseClouds()
{
	string pfad = "Test/AllDenseClouds.txt";
	cout << pfad;
	ofstream outfile(pfad);

	for (int cam1 = 0; cam1 < projection_matrices.size(); cam1++)
	{
		for (int cam2 = 0; cam2 < projection_matrices.size(); cam2++)
		{
			if (denseClouds.count(make_pair(cam1, cam2)))
			{
				for (int p = 0; p < denseClouds[make_pair(cam1, cam2)].cols; p++)
				{
					outfile << denseClouds[make_pair(cam1, cam2)].at<cv::Vec3d>(p)[0] << " ";
					outfile << denseClouds[make_pair(cam1, cam2)].at<cv::Vec3d>(p)[1] << " ";
					outfile << denseClouds[make_pair(cam1, cam2)].at<cv::Vec3d>(p)[2] << " ";
					outfile << static_cast<int>(denseCloudsColor[make_pair(cam1, cam2)].at<cv::Vec3b>(p).val[2]) << " ";
					outfile << static_cast<int>(denseCloudsColor[make_pair(cam1, cam2)].at<cv::Vec3b>(p).val[1]) << " ";
					outfile << static_cast<int>(denseCloudsColor[make_pair(cam1, cam2)].at<cv::Vec3b>(p).val[0]) << " ";
					outfile << "\n";
				}
			}
		}
	}

	outfile.close();
}






void PointCloudTable::push_denseCloud(int _cam1, int _cam2, cv::Mat R1_rect, vector<cv::Point3d> _denseCloud, vector<cv::Vec3b> _denseCloudColor)
{
	denseClouds[make_pair(_cam1, _cam2)] = cv::Mat(cv::Size(_denseCloud.size(), 1), CV_64FC3);
	denseCloudsColor[make_pair(_cam1, _cam2)] = cv::Mat(cv::Size(_denseCloudColor.size(), 1), CV_8UC3);

	cv::Mat proj_mat_temp = projection_matrices.at(_cam1).inv();
	cv::Mat R_hom_inv = R1_rect.inv();


	cv::Mat data_temp = cv::Mat(cv::Size(4, 1), CV_32FC1);
	cv::Mat data = cv::Mat(cv::Size(3, 1), CV_32FC1);

	for (int p = 0; p < _denseCloud.size(); p++)
	{
		//cv::Mat point = (cv::Mat_<double>(4, 1) <<
		//	_denseCloud.at(p).x,
		//	_denseCloud.at(p).y,
		//	_denseCloud.at(p).z,
		//	1);

		//irgendwas stimmt hier nicht.. es kommt ein laufzeitfehler manchmal bei Mat deallocate...

		data_temp.at<double>(0) = 
			R_hom_inv.at<double>(0, 0) * _denseCloud.at(p).x
			+ R_hom_inv.at<double>(0, 1) * _denseCloud.at(p).y
			+ R_hom_inv.at<double>(0, 2) * _denseCloud.at(p).z;

		data_temp.at<double>(1) =
			R_hom_inv.at<double>(1, 0) * _denseCloud.at(p).x
			+ R_hom_inv.at<double>(1, 1) * _denseCloud.at(p).y
			+ R_hom_inv.at<double>(1, 2) * _denseCloud.at(p).z;

		data_temp.at<double>(2) =
			R_hom_inv.at<double>(2, 0) * _denseCloud.at(p).x
			+ R_hom_inv.at<double>(2, 1) * _denseCloud.at(p).y
			+ R_hom_inv.at<double>(2, 2) * _denseCloud.at(p).z;

		data_temp.at<double>(3) = 1;


		/*
		cv::Mat data_temp = (cv::Mat_<double>(4, 1) <<

		  R_hom_inv.at<double>(0, 0) * _denseCloud.at(p).x
		+ R_hom_inv.at<double>(0, 1) * _denseCloud.at(p).y
		+ R_hom_inv.at<double>(0, 2) * _denseCloud.at(p).z,

		  R_hom_inv.at<double>(1, 0) * _denseCloud.at(p).x
		+ R_hom_inv.at<double>(1, 1) * _denseCloud.at(p).y
		+ R_hom_inv.at<double>(1, 2) * _denseCloud.at(p).z,

		  R_hom_inv.at<double>(2, 0) * _denseCloud.at(p).x
		+ R_hom_inv.at<double>(2, 1) * _denseCloud.at(p).y
		+ R_hom_inv.at<double>(2, 2) * _denseCloud.at(p).z, 
			
		  1);
		*/
		

		data.at<double>(0) =
			proj_mat_temp.at<double>(0, 0) * data_temp.at<double>(0) +
			proj_mat_temp.at<double>(0, 1) * data_temp.at<double>(1) +
			proj_mat_temp.at<double>(0, 2) * data_temp.at<double>(2) +
			proj_mat_temp.at<double>(0, 3);

		data.at<double>(1) =
			proj_mat_temp.at<double>(1, 0) * data_temp.at<double>(0) +
			proj_mat_temp.at<double>(1, 1) * data_temp.at<double>(1) +
			proj_mat_temp.at<double>(1, 2) * data_temp.at<double>(2) +
			proj_mat_temp.at<double>(1, 3);

		data.at<double>(2) =
			proj_mat_temp.at<double>(2, 0) * data_temp.at<double>(0) +
			proj_mat_temp.at<double>(2, 1) * data_temp.at<double>(1) +
			proj_mat_temp.at<double>(2, 2) * data_temp.at<double>(2) +
			proj_mat_temp.at<double>(2, 3);

		data.at<double>(3) =
			1;


		//cv::Mat data = cv::Mat(proj_mat_temp * data_temp);




		denseClouds[make_pair(_cam1, _cam2)].at<cv::Vec3d>(0, p)[0] = data.at<double>(0);
		denseClouds[make_pair(_cam1, _cam2)].at<cv::Vec3d>(0, p)[1] = data.at<double>(1);
		denseClouds[make_pair(_cam1, _cam2)].at<cv::Vec3d>(0, p)[2] = data.at<double>(2);
		
				
		denseCloudsColor[make_pair(_cam1, _cam2)].at<cv::Vec3b>(0, p) = _denseCloudColor.at(p);

	}

}







void PointCloudTable::writePointCloud(int view1, int view2)
{
	string pfad = "Test/test" + to_string(i++) + "_" + to_string(view1) + "," + to_string(view2) + ".ply";
	cout << pfad;
	ofstream outfile(pfad);

	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << points3d_measured.size() + projection_matrices.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n property uchar red\n" << "property uchar green\n" << "property uchar blue\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";

	
	for (int i = 0; i < points3d_measured.size(); i++)
	{
		cv::Point3d point = points3d_measured.at(i).XYZ;

		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << points3d_measured.at(i).RGB[0] << " ";
		outfile << points3d_measured.at(i).RGB[1] << " ";
		outfile << points3d_measured.at(i).RGB[2] << " ";
		outfile << "\n";
	}
	

	for (int i = 0; i < projection_matrices.size(); i++)
	{
		cv::Mat projection_matrix_inv = cv::Mat(projection_matrices.at(i).inv());

		cv::Mat R_rod;

		cv::Matx33d rot_mat(
			projection_matrix_inv.at<double>(0, 0), projection_matrix_inv.at<double>(0, 1), projection_matrix_inv.at<double>(0, 2),
			projection_matrix_inv.at<double>(1, 0), projection_matrix_inv.at<double>(1, 1), projection_matrix_inv.at<double>(1, 2),
			projection_matrix_inv.at<double>(2, 0), projection_matrix_inv.at<double>(2, 1), projection_matrix_inv.at<double>(2, 2));

		cv::Rodrigues(rot_mat, R_rod);


		//cout << "R_rod" << i << ": " << endl << R_rod << endl << endl;

		double x = projection_matrix_inv.at<double>(0, 3);
		double y = projection_matrix_inv.at<double>(1, 3);
		double z = projection_matrix_inv.at<double>(2, 3);
		outfile << x << " ";
		outfile << y << " ";
		outfile << z << " ";
		outfile << i << " ";
		outfile << 255 << " ";
		outfile << 255 << " ";
		outfile << "\n";
	}

	outfile.close();
}



void PointCloudTable::writeBALProblem(vector<Bild> bilder)
{
	ofstream outfile("BalProblem.txt");

	outfile.flags(ios::scientific);
	outfile.precision(10);




	outfile << projection_matrices.size() << " " << points3d_measured.size() << " " << observations_anzahl << "\n";

	for (int i = 0; i < points3d_measured.size(); i++)
	{
		for (int j = 0; j < points3d_measured.at(i).imgs_pnts.size(); j++)
		{
			if (points3d_measured.at(i).imgs_pnts.at(j) != -1)
			{
				outfile 
					<< j << " " 
					<< i << " " 
					<< bilder.at(j).keypoints.at(points3d_measured.at(i).imgs_pnts.at(j)).pt.x << " "
					<< bilder.at(j).keypoints.at(points3d_measured.at(i).imgs_pnts.at(j)).pt.y << endl;
			}
		}
	}
	
	for (int i = 0; i < projection_matrices.size(); i++)
	{


		cv::Mat R = (cv::Mat_<double>(3,3) <<
			projection_matrices.at(i).at<double>(0, 0), projection_matrices.at(i).at<double>(0, 1), projection_matrices.at(i).at<double>(0, 2),
			projection_matrices.at(i).at<double>(1, 0), projection_matrices.at(i).at<double>(1, 1), projection_matrices.at(i).at<double>(1, 2),
			projection_matrices.at(i).at<double>(2, 0), projection_matrices.at(i).at<double>(2, 1), projection_matrices.at(i).at<double>(2, 2));
 

		cv::Mat R_vec;
		
		cv::Rodrigues(R, R_vec);

		outfile << R_vec.at<double>(0) << endl << R_vec.at<double>(1) << endl << R_vec.at<double>(2) << endl;
		cout << endl << endl << "Jetzt kommen die R_vec" << i << ": " << R_vec << endl;
		outfile << projection_matrices.at(i).at<double>(0, 3) << endl << projection_matrices.at(i).at<double>(1, 3) << endl << projection_matrices.at(i).at<double>(2, 3) << endl;

		outfile << bilder.at(0).K.at<double>(0, 0) << endl << bilder.at(0).distCoeffs.at<double>(0, 0) << endl << bilder.at(0).distCoeffs.at<double>(1, 0) << endl;

	}

	for (int i = 0; i < points3d_measured.size(); i++)
	{
		outfile << points3d_measured.at(i).XYZ.x << endl << points3d_measured.at(i).XYZ.y << endl << points3d_measured.at(i).XYZ.z << endl;
	}




	outfile.close();
}