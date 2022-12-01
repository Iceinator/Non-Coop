#pragma once
#include <stdio.h> 
#include <iostream> 
#include <cmath>
#include<opencv2/opencv.hpp> 
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/calib3d.hpp"
using namespace cv;
using namespace std;
class Sat {
public:
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<DMatch> matched_keypoints;
	Mat descriptor1;
	Mat descriptor2;

	//Intrinsic
	Mat cameraMatrix = (Mat_<double>(3, 3) << 4766.92461024042, 0, 602.849676330207,
												0, 4733.36134108249, 569.156279267313,
												0, 0, 1);
	Mat distCoeffs = (Mat_<double>(5, 1) << 1.1682938974656, -31.6276409521339, 0, 0, 0);
	

	Point2f CenterPoint;//Centerpoint in imageplane
	Point3f Rotation;//Rotation in x,y,z
	Point3f CpOpj;//Centerpoint in object
	float RefDist; //Distance in refimg

};