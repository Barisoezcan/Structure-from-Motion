#include "Visualizer.h"




void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{

	if (event.getPointIndex() == -1)
	{
		return;
	}
	std::cout << event.getPointIndex() << endl;

}

void mcallback(const cv::viz::MouseEvent& event, void* v)
{
	if (event.button == event.LeftButton)
		std::cout << event.pointer << endl;
	return;

}


vector<PointXYZ> selected_points;


vector<pcl::visualization::Camera> cam;


void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	float x, y, z;
	if (event.getPointIndex() == -1)
	{
		return;
	}
	event.getPoint(x, y, z);
	event.getPointIndex();
	//cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << endl;
	selected_points.push_back(PointXYZ(x, y, z));

	float xd = selected_points.at(selected_points.size() - 1).x - selected_points.at(selected_points.size() - 2).x;
	float yd = selected_points.at(selected_points.size() - 1).y - selected_points.at(selected_points.size() - 2).y;
	float zd = selected_points.at(selected_points.size() - 1).z - selected_points.at(selected_points.size() - 2).z;

	//cout << "xd: " << xd << ", yd: " << yd << ", zd: " << zd << endl;

	//cout << "Point distance: " << sqrt(pow(xd, 2) + pow(yd, 2) + pow(zd, 2)) << endl;



	cout << "Cam: " << endl
		<< " - pos: (" << cam[0].pos[0] << ", " << cam[0].pos[1] << ", " << cam[0].pos[2] << ")" << endl
		<< " - view: (" << cam[0].view[0] << ", " << cam[0].view[1] << ", " << cam[0].view[2] << ")" << endl
		<< " - focal: (" << cam[0].focal[0] << ", " << cam[0].focal[1] << ", " << cam[0].focal[2] << ")" << endl;

}




inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb)
{
	pcl::PointXYZRGB p(rgb[0], rgb[1], rgb[2]);
	p.x = v[0];
	p.y = v[1];
	p.z = v[2];
	return p;
}

pcl::PolygonMesh visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, float r, float g, float b,
	Eigen::Vector3f& vforward, Eigen::Vector3f& rgb, double s = 0.01)
{
	Eigen::Vector3f vright = R.row(0).normalized() * s;
	Eigen::Vector3f vup = -R.row(1).normalized() * s;
	vforward = R.row(2).normalized() * s;

	rgb = Eigen::Vector3f(r, g, b);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(Eigen2PointXYZRGB(t, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright / 2.0 + vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright / 2.0 - vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright / 2.0 + vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright / 2.0 - vup / 2.0, rgb));

	pcl::PolygonMesh pm;
	pm.polygons.resize(6);



	return pm;
}




//PCL-Visualizer
void visualizePCL(vector<measured_3d> points3d_measured, vector<Bild> bilder, vector<cv::Mat> pmats)
{

	//pcl::visualization::CloudViewer viewer("Cloud Viewer");

	pcl::visualization::PCLVisualizer viewer2("Matrix transformation example");


	viewer2.addCoordinateSystem(3.0, "cs", 0);

	selected_points.push_back(PointXYZ(0, 0, 0));
	selected_points.push_back(PointXYZ(0, 0, 0));

	PointCloud<PointXYZRGB>::Ptr pointCloudptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < points3d_measured.size(); i++)
	{
		PointXYZRGB point;
		point.x = points3d_measured.at(i).XYZ.x;
		point.y = points3d_measured.at(i).XYZ.y;
		point.z = points3d_measured.at(i).XYZ.z;
		point.r = points3d_measured.at(i).RGB[0];
		point.g = points3d_measured.at(i).RGB[1];
		point.b = points3d_measured.at(i).RGB[2];

		pointCloudptr->points.push_back(point);
	}




	double A11 = 0, A12 = 0, A13 = 0, A21 = 0, A22 = 0, A23 = 0, A31 = 0, A32 = 0, A33 = 0;
	double B11 = 0, B12 = 0, B13 = 0;

	for (int i = 0; i < pointCloudptr->size(); i++)
	{
		A11 += points3d_measured.at(i).XYZ.x * points3d_measured.at(i).XYZ.x;
		A12 += points3d_measured.at(i).XYZ.x * points3d_measured.at(i).XYZ.y;
		A13 += points3d_measured.at(i).XYZ.x;
		A21 += points3d_measured.at(i).XYZ.x * points3d_measured.at(i).XYZ.y;
		A22 += points3d_measured.at(i).XYZ.y * points3d_measured.at(i).XYZ.y;
		A23 += points3d_measured.at(i).XYZ.y;
		A31 += points3d_measured.at(i).XYZ.x;
		A32 += points3d_measured.at(i).XYZ.y;

		B11 += points3d_measured.at(i).XYZ.x * points3d_measured.at(i).XYZ.z;
		B12 += points3d_measured.at(i).XYZ.y * points3d_measured.at(i).XYZ.z;
		B13 += points3d_measured.at(i).XYZ.z;
	}
	A33 = pointCloudptr->size();


	cv::Mat A = (cv::Mat_<double>(3, 3) << A11, A12, A13, A21, A22, A23, A31, A32, A33);

	cv::Mat B = (cv::Mat_<double>(3, 1) << B11, B12, B13);

	cv::Mat X_;

	cv::solve(A, B, X_, cv::QT_RADIOBOX);


	cout << "X_: " << X_ << endl << endl;










	vector<pcl::Vertices> polys;
	pcl::Vertices v1, v2;
	v1.vertices.push_back(0);
	v1.vertices.push_back(1);
	v1.vertices.push_back(2);
	v2.vertices.push_back(2);
	v2.vertices.push_back(1);
	v2.vertices.push_back(0);

	polys.push_back(v1);
	polys.push_back(v2);


	pcl::PointXYZ pt0, pt1, pt2, pt3;
	pt0 = PointXYZ(0, 0, 0);
	pt1 = PointXYZ(0, 1, 1);
	pt2 = PointXYZ(1, 1, 0);
	pt3 = PointXYZ(1, 0, 1);
	//fill  pt0, pt1,..., pt3 with your values!

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(pt0);
	cloud->push_back(pt1);
	cloud->push_back(pt2);
	cloud->push_back(pt3);

	pcl::PolygonMesh mesh;
	mesh.polygons = polys;
	//convert cloud to blob
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *cloud_blob);

	mesh.cloud = *cloud_blob;

	std::string ply_filename("mesh.ply");
	pcl::io::savePLYFile(ply_filename, mesh);
















	viewer2.addPointCloud(pointCloudptr);


	//viewer.showCloud(pointCloudptr);
	viewer2.registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer2);



	while (!viewer2.wasStopped())
	{
		viewer2.getCameras(cam);



		viewer2.spinOnce();
	}

}


//VIZ-Visualizer
void visualizeVIZ(PointCloudTable* pct, map<pair<int, int>, Bildpaar> bildpaare, vector<Bild> bilder)
{
	//Erzeuge Fenster
	cv::viz::Viz3d myWindow("3D Model");

	//MouseCallback ans Fenster
	myWindow.registerMouseCallback(mcallback, (void*)&myWindow);

	//Erzeuge Koordinatensystemachsen
	cv::viz::WCoordinateSystem wcs(1.0);





	//Erstelle duenne Punktwolke
	cv::Mat sparseCloud(cv::Size(pct->points3d_measured.size(), 1), CV_64FC3);
	cv::Mat sparseCloudColor(cv::Size(pct->points3d_measured.size(), 1), CV_8UC3);

	for (int i = 0; i < pct->points3d_measured.size(); i++)
	{
		sparseCloud.at<cv::Vec3d>(0, i) = cv::Vec3d(pct->points3d_measured.at(i).XYZ.x, pct->points3d_measured.at(i).XYZ.y, pct->points3d_measured.at(i).XYZ.z);
		sparseCloudColor.at<cv::Vec3b>(0, i) = cv::Vec3b(pct->points3d_measured.at(i).RGB[2], pct->points3d_measured.at(i).RGB[1], pct->points3d_measured.at(i).RGB[0]);
	}

	//Erstelle WCloud-Objekt mit duenner Punktwolke mit Farben
	cv::viz::WCloud w_sparseCloud(sparseCloud, sparseCloudColor);





	/*
	
	
	//Erstelle dichte Punktwolken
	cv::Mat denseClouds_rect(cv::Size(1, 1), CV_64FC3);
	cv::Mat denseCloudColors(cv::Size(1, 1), CV_8UC3);




	for (int i = 0; i < pct->projection_matrices.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			if (bildpaare.find(make_pair(j, i)) != bildpaare.end())
			{
				//cv::Mat P_rel = pct->projection_matrices.at(i) * pct->projection_matrices.at(j).inv();//bilder.at(i).P * bilder.at(j).P.inv();

				cout << "Bildpaar " << j << ", " << i << endl;

				cv::Mat proj_mat_temp = pct->projection_matrices.at(j).inv();


				cout << "proj_mat_temp: " << endl << proj_mat_temp << endl << endl << "proj_mat_temp.inv: " << endl << proj_mat_temp.inv() << endl << endl;

				cv::Mat R_hom = (cv::Mat_<double>(4, 4) <<
					bildpaare[make_pair(j, i)].R1_rect.at<double>(0, 0), bildpaare[make_pair(j, i)].R1_rect.at<double>(0, 1), bildpaare[make_pair(j, i)].R1_rect.at<double>(0, 2), 0,
					bildpaare[make_pair(j, i)].R1_rect.at<double>(1, 0), bildpaare[make_pair(j, i)].R1_rect.at<double>(1, 1), bildpaare[make_pair(j, i)].R1_rect.at<double>(1, 2), 0,
					bildpaare[make_pair(j, i)].R1_rect.at<double>(2, 0), bildpaare[make_pair(j, i)].R1_rect.at<double>(2, 1), bildpaare[make_pair(j, i)].R1_rect.at<double>(2, 2), 0,
					0, 0, 0, 1);




				cv::Mat R_hom_inv = R_hom.inv();



				denseClouds_rect.resize(bildpaare[make_pair(j, i)].denseCloud.size());
				denseCloudColors.resize(bildpaare[make_pair(j, i)].denseCloud_color.size());


				for (int p = 0; p < bildpaare[make_pair(j, i)].denseCloud.size(); p++)
				{


					cv::Mat point = (cv::Mat_<double>(4, 1) <<
						bildpaare[make_pair(j, i)].denseCloud.at(p).x,
						bildpaare[make_pair(j, i)].denseCloud.at(p).y,
						bildpaare[make_pair(j, i)].denseCloud.at(p).z,
						1);



					cv::Mat data = cv::Mat(proj_mat_temp * (R_hom_inv * point));


					cv::Mat data = (cv::Mat_<double>(4, 1) <<
						R_hom_inv.at<double>(0, 0) * bildpaare[make_pair(j, i)].denseCloud.at(p).x
						+ R_hom_inv.at<double>(0, 1) * bildpaare[make_pair(j, i)].denseCloud.at(p).y
						+ R_hom_inv.at<double>(0, 2) * bildpaare[make_pair(j, i)].denseCloud.at(p).z,

						R_hom_inv.at<double>(1, 0) * bildpaare[make_pair(j, i)].denseCloud.at(p).x
						+ R_hom_inv.at<double>(1, 1) * bildpaare[make_pair(j, i)].denseCloud.at(p).y
						+ R_hom_inv.at<double>(1, 2) * bildpaare[make_pair(j, i)].denseCloud.at(p).z,

						R_hom_inv.at<double>(2, 0) * bildpaare[make_pair(j, i)].denseCloud.at(p).x
						+ R_hom_inv.at<double>(2, 1) * bildpaare[make_pair(j, i)].denseCloud.at(p).y
						+ R_hom_inv.at<double>(2, 2) * bildpaare[make_pair(j, i)].denseCloud.at(p).z);
	





					denseClouds_rect.at<cv::Vec3d>(0, p) = cv::Vec3d(data.at<double>(0), data.at<double>(1), data.at<double>(2));
					denseCloudColors.at<cv::Vec3b>(0, p) = cv::Vec3b(bildpaare[make_pair(j, i)].denseCloud_color.at(p) * double((j+1) / pct->projection_matrices.size()));

				}

				w_denseClouds.push_back(cv::viz::WCloud(denseClouds_rect, denseCloudColors));

			}
		}
	}

	*/



	vector<cv::viz::WCloud> w_denseClouds;

	for (int cam2 = 0; cam2 < pct->bild_anzahl; cam2++)
	{
		for (int cam1 = 0; cam1 < pct->bild_anzahl; cam1++)
		{
			if(pct->denseClouds.count(make_pair(cam1, cam2)))
			{
				w_denseClouds.push_back(cv::viz::WCloud(pct->denseClouds[make_pair(cam1, cam2)], pct->denseCloudsColor[make_pair(cam1, cam2)]));
			}
		}
	}






	/*

	int i = 0;
	for (map<pair<int, int>, Bildpaar>::iterator it_bildpaare = bildpaare.begin(); it_bildpaare != bildpaare.end(); ++it_bildpaare, ++i)
	{
		for (vector<cv::Point3d>::iterator it_points = it_bildpaare->second.denseCloud.begin(); it_points != it_bildpaare->second.denseCloud.end(); ++it_points)
		{
			double *data = cv::Mat((pct->projection_matrices_inv.at(it_bildpaare->second.Bild1.Bild_nr) * it_bildpaare->second.R1_rect) * cv::Mat(*it_points)).ptr<double>(0);
			denseClouds_rect.push_back(cv::Point3d(data[0], data[1], data[2]));
		}
		for (vector<cv::Vec3b>::iterator it_pointColors = it_bildpaare->second.denseCloud_color.begin(); it_pointColors != it_bildpaare->second.denseCloud_color.end(); ++it_pointColors)
		{
			denseCloudColors.push_back(cv::Mat(*it_pointColors));
		}

		//Erstelle WCloud-Objekte mit dichten Punktwolken mit Farben
		w_denseClouds.push_back(cv::viz::WCloud(denseClouds_rect, denseCloudColors));
	}
	*/


	/*
	cv::Mat A(cv::Size(3, denseClouds.at(0).rows), CV_64FC1);
	cv::Mat B(cv::Size(1, denseClouds.at(0).rows), CV_64FC1);

	cv::Mat X(cv::Size(denseClouds.size(), 3), CV_64FC1);



	//Iteriere ueber dichte Punktwolken
	for (int i = 0; i < bildpaare.; it++)
	{
		for (int j = )
			cv::Mat denseCloud_temp(cv::Size(bildpaarsecond.denseCloud.size(), 1), CV_64FC3);
		cv::Mat denseCloudColor_temp(cv::Size(it->second.denseCloud_color.size(), 1), CV_8UC3);

		cv::Mat transMat = cv::Mat(transformationMatrizes.at(i));



			//p_denseClouds.at(i).at(j).x = p_denseClouds.at(i).at(j).x -p_translations.at(i).at<double>(0);
			//p_denseClouds.at(i).at(j).y = p_denseClouds.at(i).at(j).y -p_translations.at(i).at<double>(1);
			//p_denseClouds.at(i).at(j).z = p_denseClouds.at(i).at(j).z -p_translations.at(i).at<double>(2);




			denseCloud_temp.at<cv::Vec3d>(0, j) = cv::Vec3d(p_denseClouds.at(i).at(j).x, p_denseClouds.at(i).at(j).y, p_denseClouds.at(i).at(j).z);
			//if (i == 0)
			{
				denseCloudColor_temp.at<cv::Vec3b>(0, j) = cv::Vec3b(p_denseCloudColors.at(i).at(j)[0], p_denseCloudColors.at(i).at(j)[1], p_denseCloudColors.at(i).at(j)[2]);

			}
			//else if(i == 1)
			//{
			//	denseCloudColor_temp.at<cv::Vec3b>(0, j) = cv::Vec3b(255, 0, p_denseCloudColors.at(i).at(j)[0]);
			//}
			//else if (i == 2)
			//{
			//	denseCloudColor_temp.at<cv::Vec3b>(0, j) = cv::Vec3b(p_denseCloudColors.at(i).at(j)[2], 255, 0);
			//}
			//denseCloudColor_temp.at<cv::Vec3b>(0, j) = cv::Vec3b(i * 255, i * 255, i * 255);


			if (i == 0)
			{
				A.at<double>(j, 0) = p_denseClouds.at(i).at(j).x;
				A.at<double>(j, 1) = p_denseClouds.at(i).at(j).y;
				A.at<double>(j, 2) = 1;

				B.at<double>(j, 0) = p_denseClouds.at(i).at(j).z;
			}

		}


	}


	cv::Mat fit = (A.t() * A).inv() * A.t() * B;


	cout << "_X_: " << endl << fit << endl << endl;

	double z_a = fit.at<double>(0, 0) * 0 + fit.at<double>(1, 0) * 0 + fit.at<double>(2, 0);




	cv::viz::WPlane plane(cv::Point3d(0, 0, z_a), cv::Vec3d(fit.at<double>(0, 0), fit.at<double>(1, 0), fit.at<double>(2, 0)), cv::Vec3d(0, 0, 1), cv::Size(10, 10));
	*/



	//Kamera-Positionen der duennen Punktwolke
	vector<cv::viz::WCameraPosition> w_cams;
	vector<cv::Affine3d> cam_poses;



	//Iteriere ueber alle Kameras
	for (int i = 0; i < bilder.size(); i++)
	{
		//Erzeuge WCameraPosition-Objekt mit K und Bild
		cv::viz::WCameraPosition w_cam(cv::Matx33d(bilder.at(i).K), bilder.at(i).image, 1.0, cv::viz::Color::white());

		w_cams.push_back(w_cam);

		//Extrahiere Kamera Rotation und Translationen aus den Projektionsmatrizen
		cv::Mat cam_rot(pct->projection_matrices_inv.at(i)(cv::Rect(0, 0, 3, 3)));
		cv::Vec3d cam_trans(pct->projection_matrices_inv.at(i).at<double>(0, 3), pct->projection_matrices_inv.at(i).at<double>(1, 3), pct->projection_matrices_inv.at(i).at<double>(2, 3));

		//Erzeuge Affine Transformationsmatrix
		cv::Affine3d cam_pose(cam_rot, cam_trans);

		cam_poses.push_back(cam_pose);
	}
	
	

	/*
	//Kamera-Position von Dense-Cloud
	vector<cv::Mat> rotations;
	vector<cv::Vec3d> translations;
	vector<cv::Affine3d> densePoses;


	//Iteriere ueber alle Projektionsmatrizen
	for (int i = 0; i < bilder.size(); i++)
	{
		//Extrahiere Rotationsmatrizen
		cv::Mat rot_dense((bilder.at(i).P(cv::Rect(0, 0, 3, 3))) * bilder.at(i).K.inv());
		cv::Mat rotation_temp = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
		rotations.push_back(rot_dense);
		//cv::Mat rot_dense(rot_dense.inv());
		//cv::Mat rot_dense((cv::Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1));

		//Extrahiere Translationsvektoren
		//(p_projectionMats.at(i)*p_K.inv());
		cv::Vec3d trans_dense(bilder.at(i).P.at<double>(0, 3), bilder.at(i).P.at<double>(1, 3), bilder.at(i).P.at<double>(2, 3));

		//Erzeuge Kamera Pose Matrix
		cv::Affine3d densePose_temp(rot_dense, trans_dense);
		densePoses.push_back(densePose_temp);

		//Ausgabe der Rotationsmatrizen und Translationsvektoren
		cout << "Rot und Trans" << i << ": " << endl << endl << rot_dense << endl << endl << trans_dense << endl << endl;
	}
	*/
	






	//Dauerschleife zum Anzeigen der Punktwolke im Viewer
	while (!myWindow.wasStopped())
	{
		//Zeige die einzelnen Kameras im Viewer
		for (int i = 0; i < w_cams.size(); i++)
		{
			myWindow.showWidget("Cam" + i, w_cams.at(i), cam_poses.at(i));
		}

		//Zeige die duenne Punktwolke im Viewer
		myWindow.showWidget("SparseCloud", w_sparseCloud);

		//Zeige die einzelnen dichten Punktwolken im Viewer
		for (int i = 0; i < w_denseClouds.size(); i++)
		{
			myWindow.showWidget(string("DenseCloud" + i), w_denseClouds.at(i));
		}

		//Zeige die Koordinatensystemachsen im Viewer
		myWindow.showWidget("wcs", wcs);

		//Zeige Ausgleichsebene im Viewer
		//myWindow.showWidget("Plane", plane);

		//Veraendere Punktwolkeneigenschaften
		myWindow.setRenderingProperty("SparseCloud", cv::viz::POINT_SIZE, 2);

		//Event loop
		myWindow.spinOnce(1, true);
	}


}