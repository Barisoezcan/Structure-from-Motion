#include "Matcher.h"

/*


void goodMatcher(vector<Mat> images, vector<vector<KeyPoint>> &keypoints, vector<Mat> &descriptors, vector<DMatch> &symmMatches, vector<matching_pair> &pairs)
{
	int minHessian = 400;

	Ptr<SURF> detector = SURF::create(minHessian);



	
	Mat descriptors_query;
	vector<Mat> descriptors_train(images.size());


	detector->detectAndCompute(images.at(0), Mat(), keypoints.at(0), descriptors_query);

	for (int i = 1; i < images.size(); i++)
	{
		detector->detectAndCompute(images.at(i), Mat(), keypoints.at(i), descriptors_train.at(i));
	}


	vector<DMatch> matches;

	Ptr<DescriptorMatcher> matcher_new = DescriptorMatcher::create("FlannBased");

	matcher_new->add(descriptors_train);
	matcher_new->train();

	matcher_new->match(descriptors_query, matches);
	


	for (int i = 0; i < images.size(); i++)
		detector->detectAndCompute(images.at(i), Mat(), keypoints.at(i), descriptors.at(i));



	cout << "d " << descriptors.at(0).rows << endl;

	BFMatcher matcher;

	matching_pair pair_temp;

	int matching_boundary = 40000;

	//old style
	for (int i = 0; i < images.size(); i++)
	{
		for (int j = i+1; j < images.size(); j++)
		{
			vector<vector<DMatch>> matches1_temp;
			vector<vector<DMatch>> matches2_temp;
			vector<DMatch> symmMatches_temp;

			cout << "i: " << i << ", j: " << j << endl;
			matcher.knnMatch(descriptors.at(i), descriptors.at(j), matches1_temp, 2);
			matcher.knnMatch(descriptors.at(j), descriptors.at(i), matches2_temp, 2);

			ratioTest(matches1_temp);
			ratioTest(matches2_temp);

			cout << "Matches1 size after ratioTest: " << matches1_temp.size() << endl;
			cout << "Matches2 size after ratioTest: " << matches2_temp.size() << endl;

			symmTest(matches1_temp, matches2_temp, symmMatches_temp);
			cout << "Matches size after symmTest: " << symmMatches_temp.size() << endl << endl;

			//Wenn #matching features hoch genug, dann images sind matches
			if (symmMatches_temp.size() > matching_boundary)
			{
				pair_temp = { i, j, keypoints.at(i), keypoints.at(j), symmMatches_temp};
				pairs.push_back(pair_temp);
			}
		}
	}

	cout << pairs.size();

	
	Mat mask (1, descriptors.size(), CV_8UC1, Scalar(1));

	for (int i = 0; i < images.size(); i++)
	{
		cout << i;
		matcher.knnMatch(descriptors.at(i), descriptors, matches1_temp, 2, mas);
		cout << ", Matches1 size first: " << matches1_temp.size() << endl;
		cout << mask;
		cout << "Matches size after symmTest: " << symmMatches_temp.size() << endl << endl << endl;
	}
	

}


void ratioTest(vector<vector<DMatch>> &matches)
{
	double ratio = 1;


	for (int i = 0; i < matches.size(); i++)
	{
		if (matches.at(i).at(0).distance / matches.at(i).at(1).distance > ratio)
		{
			matches.at(i).clear();
		}

		
	}
}



void symmTest(vector<vector<DMatch>> matches1, vector<vector<DMatch>> matches2, vector<DMatch> &symmMatches)
{
	cout << "symmMatches in funktion: " << symmMatches.size() << endl;

	for (int i = 0; i < matches1.size(); i++)
	{
		if (matches1.at(i).size() < 2)
		{
			continue;
		}
		for (int j = 0; j < matches2.size(); j++)
		{
			if (matches2.at(j).size() < 2)
			{
				continue;
			}
			if ((matches1.at(i).at(0).trainIdx == matches2.at(j).at(0).queryIdx) && (matches1.at(i).at(0).queryIdx == matches2.at(j).at(0).trainIdx))
			{
				symmMatches.push_back(DMatch(matches1.at(i).at(0).queryIdx, matches1.at(i).at(0).trainIdx, matches1.at(i).at(0).distance));
				break;
			}
		}
	}
}



void makePairs(vector<Mat> &descriptors, vector<vector<DMatch>> &matches, vector<vector<int>> &pairs)
{
	FlannBasedMatcher matcher;

	for (int i = 0; i < descriptors.size(); i++)
	{
		for (int j = 0; j < descriptors.size(); j++)
		{
			if (i != j){
				cout << "Matching " << i << " (" << descriptors.at(i).size() << ") with " << j << " (" << descriptors.at(j).size() << "): ";
				matcher.knnMatch(descriptors.at(i), descriptors.at(j), matches.at(i), 2);
				cout << "matches.size: " << matches.at(j).size() << endl;
			}
		}
	}
}





static void maskMatchesByTrainImgIdx(const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask)
{
	mask.resize(matches.size());
	fill(mask.begin(), mask.end(), 0);
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (matches[i].imgIdx == trainImgIdx)
			mask[i] = 1;
	}
}



static void saveResultImages(const Mat& queryImage, const vector<KeyPoint>& queryKeypoints,
	const vector<Mat>& trainImages, const vector<vector<KeyPoint> >& trainKeypoints,
	const vector<DMatch>& matches, const vector<string>& trainImagesNames, const string& resultDir)
{
	cout << "< Save results..." << endl;
	Mat drawImg;
	vector<char> mask;
	for (size_t i = 0; i < trainImages.size(); i++)
	{
		if (!trainImages[i].empty())
		{
			maskMatchesByTrainImgIdx(matches, (int)i, mask);
			drawMatches(queryImage, queryKeypoints, trainImages[i], trainKeypoints[i],
				matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask);
			string filename = resultDir + "/res_" + trainImagesNames[i];
			if (!imwrite(filename, drawImg))
				cout << "Image " << filename << " can not be saved (may be because directory " << resultDir << " does not exist)." << endl;
		}
	}
	cout << ">" << endl;
}




void calculateFundamentalMatrices(vector<matching_pair> pairs)
{

	for (int i = 0; i < pairs.size(); i++)
	{
		vector<Point2f> points1, points2;

		for (int m = 0; m < pairs.at(i).matches.size(); m++)
		{
			points1.push_back(pairs.at(i).keypoints1.at(pairs.at(i).matches.at(m).queryIdx).pt);
			points2.push_back(pairs.at(i).keypoints2.at(pairs.at(i).matches.at(m).trainIdx).pt);
		}

		pairs.at(i).points1 = points1;
		pairs.at(i).points2 = points2;
		pairs.at(i).F = findFundamentalMat(points1, points2);

	}
}

*/