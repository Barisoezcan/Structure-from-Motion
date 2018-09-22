#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>


#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "PointCloudTable.h"


using namespace pcl;

void visualizePCL(vector<measured_3d>, vector<Bild>, vector<cv::Mat>);

void visualizeVIZ(PointCloudTable* pct, map<pair<int, int>, Bildpaar>, vector<Bild> bilder);