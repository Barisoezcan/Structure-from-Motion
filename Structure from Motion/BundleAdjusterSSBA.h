/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include <vector>
#include <opencv2/core/core.hpp>
#include "PointCloudTable.h"


#define V3DLIB_ENABLE_SUITESPARSE

#include <Math/v3d_linear.h>
#include <Base/v3d_vrmlio.h>
#include <Geometry/v3d_metricbundle.h>

using namespace V3D;

class BundleAdjuster {
public:
	void adjustBundle(vector<measured_3d>& pointcloud,
		cv::Mat& cam_matrix,
		vector<Bild>,
		std::vector<cv::Mat>& Pmats);
private:
	int Count2DMeasurements(const std::vector<measured_3d>& pointcloud);
};
