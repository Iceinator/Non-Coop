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

	Point2f CenterPoint;//Centerpoint in imageplane
	Point3f Rotation;//Rotation in x,y,z
	Point3f CpOpj;//Centerpoint in object
	float RefDist; //Distance in refimg



};