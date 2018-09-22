#include "Model.h"

#include "pba.h"
//#include "util.h"

#include <opencv2\sfm.hpp>


Model::Model(vector<Bild> _bilder)
{
	bilder = _bilder;


	createBildpaare();

	createModel();
}



void Model::createBildpaare()
{
	cout << endl << endl << "Erstelle die Bildpaare ________________________________________" << endl << endl;

	for (int i = 0; i < bilder.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{

			Bildpaar bildpaar_temp(&(bilder.at(j)), &(bilder.at(i)));
			if (bildpaar_temp.isPaar)
			{
				bildpaare[make_pair(j, i)] = bildpaar_temp;
				cout << "Bildpaar(" << j << "," << i << ") ist ein Paar und hat " << bildpaar_temp.guteMatches.size() << " Matches." << endl;
			}
			else
			{
				cout << "Bildpaar(" << j << "," << i << ") ist kein Paar und hat nur " << bildpaar_temp.guteMatches.size() << " Matches." << endl;
			}
		}
	}
	cout << endl << endl << "Bildpaare erfolgreich erstellt! _________________________________" << endl << endl;
}



/*
void Model::insertImage(Bild bild)
{
	vector<Bildpaar> bildpaare_temp;

	bilder.push_back(bild);

	for (int i = 0; i < bilder.size(); i++)
	{
		Bildpaar bildpaar(bild, bilder.at(i));

		if (bildpaar.isPaar)
		{
			bildpaare_temp.push_back(bildpaar);
		}
	}


}
*/




void Model::createModel()
{
	cout << endl << endl << "Erstelle das initiale Modell _____________________________" << endl << endl;


	int bild_anzahl = bilder.size();
	pct = new PointCloudTable(bild_anzahl);


	cv::Mat K = bilder.at(0).K;

	//Initialize
	int bild_nr1 = 0;
	int bild_nr2 = 1;


	cv::Mat P_init = (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);

	cv::Mat P1 = (cv::Mat_<double>(4, 4) <<
		bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(0, 0), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(0, 1), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(0, 2), bildpaare[make_pair(bild_nr1, bild_nr2)].t.at<double>(0),
		bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(1, 0), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(1, 1), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(1, 2), bildpaare[make_pair(bild_nr1, bild_nr2)].t.at<double>(1),
		bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(2, 0), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(2, 1), bildpaare[make_pair(bild_nr1, bild_nr2)].R.at<double>(2, 2), bildpaare[make_pair(bild_nr1, bild_nr2)].t.at<double>(2),
		0, 0, 0, 1);



	cout << "P0: " << endl << P_init << endl << endl;
	cout << "P1: " << endl << P1 << endl << endl;

	vector<cv::Point3d> new_points3d;

	new_points3d = triangulation.LinearTriangulation(bildpaare[make_pair(bild_nr1, bild_nr2)].Bild1.K, P_init, P1, bildpaare[make_pair(bild_nr1, bild_nr2)].image_points1, bildpaare[make_pair(bild_nr1, bild_nr2)].image_points2);

	cv::Mat R;
	cv::Mat R_1, t_1;
	cv::solvePnP(new_points3d, bildpaare[make_pair(bild_nr1, bild_nr2)].image_points2, K, cv::Mat(), R_1, t_1);

	cv::Rodrigues(R_1, R);

	cv::Mat P_t = (cv::Mat_<double>(4, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t_1.at<double>(0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t_1.at<double>(1),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t_1.at<double>(2),
		0, 0, 0, 1);


	pct->projection_matrices.push_back(P_init);
	pct->projection_matrices.push_back(P_t);
	pct->projection_matrices_inv.push_back(P_init.inv());
	pct->projection_matrices_inv.push_back(P_t.inv());
	//bilder.at(0).P = P_init.inv();
	//bilder.at(1).P = P_t.inv();


	cout << "P_init: " << endl << P_init << endl << endl;
	cout << "P_t: " << endl << P_t << endl << endl;

	cout << "Es wurden " << bildpaare[make_pair(bild_nr1, bild_nr2)].image_points1.size() << " Punkte trianguliert!" << endl;

	//Push die Beobachtungen
	pct->push_measurement_init(bild_nr1, bild_nr2, bildpaare[make_pair(bild_nr1, bild_nr2)].Bild1.image, bildpaare[make_pair(bild_nr1, bild_nr2)].Bild2.image, new_points3d, bildpaare[make_pair(bild_nr1, bild_nr2)].Bild1.keypoints, bildpaare[make_pair(bild_nr1, bild_nr2)].Bild2.keypoints, bildpaare[make_pair(bild_nr1, bild_nr2)].guteMatches, bildpaare);

	//BA.adjustBundle(pct.points3d_measured, K, bilder, pct.projection_matrices_inv);

	pct->writePointCloud(bild_nr1, bild_nr2);

	done_view.push_back(bild_nr1);
	done_view.push_back(bild_nr2);





	cout << endl << endl << "Initiales Modell erfolgreich erstellt! _____________________" << endl << endl;



	cout << endl << endl << "Erweitere das initiale Modell _______________________________" << endl << endl;


	for (int working_view = 2; working_view < bild_anzahl; working_view++)
	{

		cout << endl << endl << "Es wird gerade mit Working_view " << working_view << " gearbeitet!" << endl;

		vector<cv::Point2d> points2d_found;
		vector<cv::Point3d> points3d_found;

		cv::Mat R_vec, t;

		pct->find_point3D_init(working_view, bilder.at(working_view).keypoints, bildpaare, points2d_found, points3d_found);

		solvePnPRansac(points3d_found, points2d_found, bilder.at(working_view).K, bilder.at(working_view).distCoeffs, R_vec, t);


		Rodrigues(R_vec, R);

		cv::Mat P_working = (cv::Mat_<double>(4, 4) <<
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
			0, 0, 0, 1);








		for (int j = 0; j < done_view.size(); j++)
		{
			if (bildpaare.find(make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)) != bildpaare.end())
			{
				cout << "\n\nTriangulation des Bildpaars (" << done_view.at(j) << ", " << bilder.at(working_view).Bild_nr << ")" << endl;

				vector<cv::Point3d> newPoints = triangulation.LinearTriangulation(bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].Bild1.K, pct->projection_matrices.at(done_view.at(j)), P_working, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].image_points1, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].image_points2);

				cout << "Es wurden " << bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].image_points1.size() << " Punkte trianguliert!" << endl;
				pct->push_measurement_init(bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].bild1_nr, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].bild2_nr, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].Bild1.image, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].Bild2.image, newPoints, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].Bild1.keypoints, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].Bild2.keypoints, bildpaare[make_pair(done_view.at(j), bilder.at(working_view).Bild_nr)].guteMatches, bildpaare);

				pct->writePointCloud(done_view.at(j), bilder.at(working_view).Bild_nr);

				//bilder.at(working_view).P = P_working;
			}

		}
		//BA.adjustBundle(pct.points3d_measured, K, bilder, pct.projection_matrices_inv);
		pct->projection_matrices.push_back(P_working);
		pct->projection_matrices_inv.push_back(P_working.inv());
		done_view.push_back(working_view);


	}



	cout << endl << endl << "Modell fertig erstellt! _____________________" << endl << endl;


	//visualize2(pct.points3d_measured, bilder, pct.projection_matrices);




	vector<int> minDisps(pct->projection_matrices.size() - 1, INT_MAX);

	int minx1 = 0, minx2 = 0, miny1 = 0, miny2 = 0, minDist = 0;
	




	vector<Mat> Rs(pct->projection_matrices.size() - 1);
	vector<Mat> ts(pct->projection_matrices.size() - 1);



	//Mat P_temp = pct.projection_matrices.at(1);
	//SM.berechnePointCloud(bildpaare[make_pair(bild_nr1, bild_nr2)].Bild1.image, bildpaare[make_pair(bild_nr1, bild_nr2)].Bild2.image, K, P_temp(Rect(0, 0, 3, 3)), P_temp(Rect(3, 0, 1, 3)), Rs.at(0), points.at(0), colours.at(0), 20, minx1, minx2, miny1, miny2);
	//ts.at(0) = pct.projection_matrices.at(0)(Rect(3, 0, 1, 3));
	//Rs.at(0) = Rs.at(0).inv();
	vector<Mat> Projs(bild_anzahl);

	
	Mat P_rel;
	Mat R_rel;
	Mat t_rel;

	
	for (int i = 0; i < pct->projection_matrices.size(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			if (bildpaare.find(make_pair(j, i)) != bildpaare.end())
			{
				for (int k = 0; k < bildpaare[make_pair(j, i)].image_points1.size(); k++)
				{
					if (bildpaare[make_pair(j, i)].image_points1.at(k).x - bildpaare[make_pair(j, i)].image_points2.at(k).x > minDist)
					{
						minx1 = bildpaare[make_pair(j, i)].image_points1.at(k).x;
						minx2 = bildpaare[make_pair(j, i)].image_points2.at(k).x;
						miny1 = bildpaare[make_pair(j, i)].image_points1.at(k).y;
						miny2 = bildpaare[make_pair(j, i)].image_points2.at(k).y;
					}
					minDist = bildpaare[make_pair(j, i)].image_points1.at(k).x - bildpaare[make_pair(j, i)].image_points2.at(k).x;
				}
				cout << "minDisp: " << minDist << ", nDisp: " << endl << endl;

				cout << "Test: " << endl << endl << bildpaare[make_pair(j, i)].Bild2.P << endl << endl << bildpaare[make_pair(j, i)].Bild1.P << endl;


				P_rel = pct->projection_matrices.at(i) * pct->projection_matrices.at(j).inv();//bilder.at(i).P * bilder.at(j).P.inv();
				R_rel = P_rel(Rect(0, 0, 3, 3));
				t_rel = P_rel(Rect(3, 0, 1, 3));

				//P_temp = P_temp.inv();
				
				test

				SM.berechnePointCloud(bildpaare[make_pair(j, i)].Bild1.image, bildpaare[make_pair(j, i)].Bild2.image, K, R_rel, t_rel, bildpaare[make_pair(j, i)].R1_rect, bildpaare[make_pair(j, i)].Bild1.P, bildpaare[make_pair(j, i)].Bild2.P, bildpaare[make_pair(j, i)].denseCloud, bildpaare[make_pair(j, i)].denseCloud_color, minx1, minx2 + 200, miny1, miny2);


				pct->push_denseCloud(j, i, bildpaare[make_pair(j, i)].R1_rect, bildpaare[make_pair(j, i)].denseCloud, bildpaare[make_pair(j, i)].denseCloud_color);


				minDist = 0;
				minx1 = 0;
				minx2 = 0;
				miny1 = 0;
				miny2 = 0;
			}
		}
	}

	//SM.berechnePointCloud(bildpaare[make_pair(1, 2)].Bild1.image, bildpaare[make_pair(1, 2)].Bild2.image, K, P_temp(Rect(0, 0, 3, 3)), P_temp(Rect(3, 0, 1, 3)), Rs.at(1),	pct->denseClouds.at(1), pct->denseClouds.at(1)colours.at(1));


	//BA.adjustBundle(pct.points3d_measured, K, bilder, pct.projection_matrices_inv);


	pct->writeAllDenseClouds();





	visualizeVIZ(pct, bildpaare, bilder);
	//visualizePCL(pct.points3d_measured, bilder, pct.projection_matrices);
		
	
}



/*
void Model::extendModel(Bild _bild)
{
	for (int i = 0; i < bilder.size(); i++)
	{
		Bildpaar bildpaar(bilder.at(i), _bild);

		if (bildpaar.isPaar)
		{
			bildpaare.push_back(bildpaar);
		}

		//finde P0, P1




		Matx34d P0, P1;
		vector<Point3d> points3d = triangulation.insertImage(bildpaare.at(i), P0, P1);

		//Push die Projektionsmatrizen
		pct.projection_matrices.push_back(P_init);
		pct.projection_matrices.push_back(P1);


		//Push die Beobachtungen
		pct.push_measurement_init(bildpaare.at(i).Bild1.Bild_nr, bildpaare.at(i).Bild2.Bild_nr, points3d, bildpaare.at(i).image_points1, bildpaare.at(i).image_points2, bildpaare.at(i).Bild1.image, bildpaare.at(i).Bild2.image);
		pct.writePointCloudEnd();


	}



	}

	*/