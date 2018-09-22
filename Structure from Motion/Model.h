#include "Triangulation.h"
#include "PointCloudTable.h"
#include "BundleAdjusterSSBA.h"
#include "Visualizer.h"
#include "StereoMatcherSGM.h"
#include <algorithm>
#include <list>
#include <utility>
#include <map>


class Model{

private:
	StereoMatcherSGM SM;
	BundleAdjuster BA;
	map<pair<int, int>, vector<Point3d>> denseClouds;
	PointCloudTable* pct;


public:
	Triangulation triangulation;

	vector<Bild> bilder;
	map<pair<int, int>, Bildpaar> bildpaare;
	vector<int> done_view;



	Model(vector<Bild>);
	void createBildpaare();
	void insertImage(Bild);
	void createModel();
	//void extendModel(Bild);


	vector<int> image_list;



};