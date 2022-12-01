#include "Header.h"
Mat rgbtogray(Mat input_img) {
	Mat greyMat;
	Mat binary_image;
	cvtColor(input_img, greyMat, COLOR_BGR2GRAY);
	threshold(greyMat, binary_image, 100, 255, THRESH_BINARY);
	return binary_image;
}

Mat CannyThreshold(Mat src_img){
	Mat src;
	src_img.copyTo(src);
	Mat dst, detected_edges;
	uint lowThreshold = 1;
	uint ratio = 1;
	uint kernel_size = 3;
	dst.create(src.size(), src.type());
    blur(src_img, detected_edges, Size(5, 5));
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    dst = Scalar::all(0);
	src.copyTo(dst, detected_edges);
	return dst;
}

void FindKeypointsRef(Mat Ref,Sat* Sat1, int n_features = 50) {
	/* Takes the refernce image and findes the 
	keypoints and descripertor for the refernce
	
	Input ref image

	Output Keypoints, Descriptor TO Sat class
	
	*/
	Mat descriptor_1;
	vector<KeyPoint> keypoints_1;
	
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	Ptr<ORB> detector = ORB::create(n_features);
	Ptr<DescriptorExtractor> extractor;

	//Find key points in ref image
	detector->detect(Ref, keypoints_1);
	detector->compute(Ref, keypoints_1, descriptor_1);
	
	//Mat img_match;
	//drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match, Scalar::all(-1), Scalar::all(-1),
		//vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//return img_match, keypoints_1, keypoints_2, descriptor_1, descriptor_2;
	
	Sat1->keypoints1 = keypoints_1;
	Sat1->descriptor1 = descriptor_1.clone();
}

void MatchKeypoints(Mat imgRef, Mat imgObs,Mat* out_img,Sat* Sat1,vector<DMatch>* matches_out, int n_features = 50) {
	/*
	Funktion that matchs the points for the ref imeage to the observed
	
	*/
	
	Mat descriptor_2;
	vector<KeyPoint> keypoints_2;
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	Ptr<ORB> detector = ORB::create(n_features);
	Ptr<DescriptorExtractor> extractor;
	
	//Find keypoints in obs image and compute descriptor
	detector->detect(imgObs, keypoints_2);
	detector->compute(imgObs, keypoints_2, descriptor_2);

	//Match key points
	matcher->match(Sat1->descriptor1, descriptor_2, matches);
	Mat img_match;
	drawMatches(imgRef, Sat1->keypoints1, imgObs, keypoints_2, matches, img_match, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//return img_match, keypoints_1, keypoints_2, descriptor_1, descriptor_2;
	*out_img = img_match.clone();
	
	Sat1->keypoints2 = keypoints_2;
	Sat1->descriptor2 = descriptor_2.clone();
	*matches_out = matches;
}

vector<Point2f> Points(vector<KeyPoint> inkeypoints) {
	vector<Point2f> out;
	KeyPoint::convert(inkeypoints,out, std::vector< int >());
	return out;
}

int homographyCalculator(vector<DMatch>* matched_keypoints,Sat* Sat1, Mat* homography, Mat* img1, Mat* img2, Mat* res,double nn_match_ratio = 1000,double ransac_thresh = 2.5f) {
	vector<KeyPoint> matched1, matched2;
	vector<DMatch> mkeypoints = *matched_keypoints;
	vector<KeyPoint> kpoints1 = Sat1->keypoints1;
	vector<KeyPoint> kpoints2 = Sat1->keypoints2;
	for (unsigned i = 0; i < mkeypoints.size(); i++) {
		//cout << mkeypoints[i].distance;
		//cout << nn_match_ratio * mkeypoints[i].distance;
		if (mkeypoints[i].distance < nn_match_ratio * mkeypoints[i].distance) {
			matched1.push_back(kpoints1[mkeypoints[i].queryIdx]);
			matched2.push_back(kpoints2[mkeypoints[i].trainIdx]);
		}
	}

	// Angles (angs) and magnitudes (mags)
	vector<float> angs(matched1.size());
	vector<float> mags(matched1.size());

	for (int i = 0; i < matched1.size(); i++) {
		angs[i] = atan((matched2[i].pt.y - matched1[i].pt.y) / (matched2[i].pt.x - matched1[i].pt.x) * 180 / 3.1415926);
		mags[i] = sqrt(pow(matched2[i].pt.y - matched1[i].pt.y, 2) + pow(matched2[i].pt.x - matched1[i].pt.x, 2));
		//cout << "Angle of matched pairs at i = " << i << ": " << setprecision(2) << angs[i] << " deg";
		//cout << ", Magnitude: " << mags[i] << endl;
	}

	float stdDev_ang = 0.0;
	float stdDev_mag = 0.0;

	for (int k = 0; k < matched1.size(); k++) {
		stdDev_ang += pow(angs[k] - mean(angs)[0], 2);
		stdDev_mag += pow(mags[k] - mean(mags)[0], 2);
	}

	stdDev_ang = sqrt(stdDev_ang / ((float)matched1.size()));
	stdDev_mag = sqrt(stdDev_mag / ((float)matched1.size()));

	//cout << "std angles: " << stdDev_ang << endl;
	//cout << "std magnitudes: " << stdDev_mag << endl;

	vector<bool> sorted_matches_ang(matched1.size());
	vector<bool> sorted_matches_mag(matched1.size());
	vector<bool> sorted_matches(matched1.size());

	for (int m = 0; m < matched1.size(); m++) {
		cout << abs(angs[m] - mean(angs)[0]) << endl;
		if (abs(angs[m] - mean(angs)[0]) < stdDev_ang) {
			sorted_matches_ang[m] = 1;
		}
		else {
			sorted_matches_ang[m] = 0;
		}
		if (abs(mags[m] - mean(mags)[0]) < stdDev_mag) {
			sorted_matches_mag[m] = 1;
		}
		else {
			sorted_matches_mag[m] = 0;
		}
		sorted_matches[m] = sorted_matches_ang[m] && sorted_matches_mag[m];

		//cout << "Sorted matches at " << m << ": " << sorted_matches_ang[m] << ", " << sorted_matches_mag[m] << ", " << sorted_matches[m] << endl;
	}

	vector<KeyPoint> matched1_;
	vector<KeyPoint> matched2_;
	for (unsigned i = 0; i < matched1.size(); i++) {
		//cout << mkeypoints[i].distance;
		//cout << nn_match_ratio * mkeypoints[i].distance;
		if (sorted_matches[i] == 1) {
			matched1_.push_back(matched1[i]);
			matched2_.push_back(matched2[i]);
		}
	}

	Mat inlier_mask, homographytemp;
	vector<KeyPoint> inliers1, inliers2;
	vector<DMatch> inlier_matches;
	if (matched1.size() >= 4) {
		homographytemp = findHomography(Points(matched1_), Points(matched2_),
			RANSAC, ransac_thresh, inlier_mask);
	}
	else {
		cout << "Error: Homography determination failed";
		return 1;
	}
	//cout << homography;

	for (unsigned i = 0; i < matched1_.size(); i++) {
		if (inlier_mask.at<uchar>(i)) {
			int new_i = static_cast<int>(inliers1.size());
			inliers1.push_back(matched1_[i]);
			inliers2.push_back(matched2_[i]);
			inlier_matches.push_back(DMatch(new_i, new_i, 0));
		}
	}
	Mat restemp;
	Mat img_1 = *img1;
	Mat img_2 = *img2;
	drawMatches(img_1, inliers1, img_2, inliers2,
		inlier_matches, restemp,
		Scalar(255, 0, 0), Scalar(255, 0, 0));
	*res = restemp;
	*homography = homographytemp;

	return 0;
}

Vec3f rotateVector(Vec3f input, Mat RS) {
	Mat input_t = Mat(input);
	Mat RS_f;
	RS.convertTo(RS_f, CV_32FC1);
	Mat rotated_vector = RS_f * input_t;
	return Vec3f(rotated_vector);
}

Point2f Cp(Mat output) {
	//Funktion Finds the center point
	
	
	// Find point of origin
	Mat thr, gray, src;

	// convert image to grayscale
	cvtColor(output, gray, COLOR_BGR2GRAY);

	// convert grayscale to binary image
	threshold(gray, thr, 100, 255, THRESH_BINARY);

	// finds first order moments of the image
	Moments m = moments(thr, true);
	Point2f p(m.m10 / m.m00, m.m01 / m.m00);
	return p;
}

Point2f CpTrack(Mat H, Point2f Cp) {
	//Finds the the center point in Ref imge and tracks to the new image
	Point2f CpNew;
	Mat d_f;
	Mat_<float> H2;
	H.copyTo(H2);
	

	Vec3f C = (0, 0, 1);
	C[0] = Cp.x;
	C[1] = Cp.y;
	C[2] = 1;
	
	//Making the dot produckt
	d_f = H2 * Mat(C);

	//Getting the values out to do cals
	float a = d_f.at<float>(0, 0);
	float b = d_f.at<float>(1, 0);
	float c = d_f.at<float>(2, 0);
	CpNew.x = a / c;
	CpNew.y = b / c;

	return CpNew;
}

int BestRotSolution(int solutions, vector<Mat> Rs_decomp, vector<Mat>ts_decomp, vector<Mat>normals_decomp,double d_inv1) {
	int best_decomp;
	double current_minimum = 500;
	for (int i = 0; i < solutions; i++)
		// Focal length = 5mm
	{
	
		double factor_d1 = 1.0/d_inv1;
		Mat rvec_decomp;
		Rodrigues(Rs_decomp[i], rvec_decomp);
		cout << "Solution " << i << ":" << endl;
		cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
		double sqrt_squares = sqrt(pow(rvec_decomp.at<double>(0, 0), 2) + pow(rvec_decomp.at<double>(1, 0), 2) + pow(rvec_decomp.at<double>(2, 0), 2));
		if (sqrt_squares <= current_minimum) {
			current_minimum = sqrt_squares;
			best_decomp = i;
		}
		cout << "Length of rotation vector: " << sqrt_squares << "\n";
		cout << "Best solution: " << best_decomp << "\n";
		//cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
		cout << "tvec from homography decomposition: " << ts_decomp[i].t() << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
		//cout << "tvec from camera displacement: " << t_1to2.t() << endl;
		cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << endl;
		//cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;


	}

	return best_decomp;
}

void DrawPOS(Mat* img2, vector<Mat> Rs_decomp,int best_decomp,Sat* Sat1) {
	
	
	Vec3f X = (1, 0, 0);
	X[0] = 1.0;
	Vec3f Y = (0, 0, 0);
	Y[1] = -1.0;
	Vec3f Z = (0, 0, 0);
	Z[2] = 1;
	Vec3f X_R = rotateVector(X, Rs_decomp[best_decomp]);
	Vec3f X_R_Scaled;
	Vec3f Y_R_Scaled;
	Vec3f Z_R_Scaled;
	X_R_Scaled[0] = X_R[0] * 50;
	X_R_Scaled[1] = X_R[1] * 50;
	X_R_Scaled[2] = X_R[2] * 50;
	Vec3f Y_R = rotateVector(Y, Rs_decomp[best_decomp]);
	Y_R_Scaled[0] = Y_R[0] * 50;
	Y_R_Scaled[1] = Y_R[1] * 50;
	Y_R_Scaled[2] = Y_R[2] * 50;
	Vec3f Z_R = rotateVector(Z, Rs_decomp[best_decomp]);
	Z_R_Scaled[0] = Z_R[0] * 50;
	Z_R_Scaled[1] = Z_R[1] * 50;
	Z_R_Scaled[2] = Z_R[2] * 50;



	//Point2f p_t = p + Point2f(ts_decomp[best_decomp].at<double>(0,0), ts_decomp[best_decomp].at<double>(1, 0));

	//Point_<int> cp = Point2i(sumX/Nkeypoint, sumY/Nkeypoint);
	Point_<int> cendx = Sat1->CenterPoint + Point2f(X_R_Scaled[0], X_R_Scaled[1]);
	Point_<int> cendy = Sat1->CenterPoint + Point2f(Y_R_Scaled[0], Y_R_Scaled[1]);
	Point_<int> cendz = Sat1->CenterPoint + Point2f(Z_R_Scaled[0], Z_R_Scaled[1]);


	arrowedLine(*img2, Sat1->CenterPoint, cendx, Scalar(255, 0, 0), 1);
	arrowedLine(*img2, Sat1->CenterPoint, cendy, Scalar(0, 255, 0), 1);
	arrowedLine(*img2, Sat1->CenterPoint, cendz, Scalar(0, 0, 255), 1);


}

vector<Point3f> calcChessboardCorners(Size boardSize,int squareSize) {
	
	vector<Point3f> objectPoints;
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			objectPoints.push_back(Point3f(float(j * squareSize),
				float(i * squareSize), 0));
	return objectPoints;
}

double Distance(Mat Skak1, Sat* Sat1,float squareSize,string intrinsicsPath) {
	//Takes two chess boards and computes the distance from the image plane to the object plan
	
	
	Size patternSize = Size(9, 6);
	vector<Point2f> corners1, corners2;
	bool found1 = findChessboardCorners(Skak1, patternSize, corners1);
	
	if (!found1)
	{
		cout << "Error, cannot find the chessboard corners in both images." << endl;
		return 1;
	}
	vector<Point3f> objectPoints;
	//calsChessboardCorners()
	string intrinsicsPathCM = intrinsicsPath + "D1";
	string intrinsicsPathDC = intrinsicsPath + "K1";
	objectPoints = calcChessboardCorners(patternSize, squareSize);
	FileStorage fsCM(samples::findFile(intrinsicsPathCM), FileStorage::READ);
	FileStorage fsDC(samples::findFile(intrinsicsPathDC), FileStorage::READ);
	Mat cameraMatrix, distCoeffs;
	

	fsCM["CM"] >> cameraMatrix;
	fsDC["DC"] >> distCoeffs;
	
	Mat rvec1, tvec1;
	solvePnP(objectPoints, corners1, cameraMatrix, distCoeffs, rvec1, tvec1);
	//Mat rvec2, tvec2;
	//solvePnP(objectPoints, corners2, cameraMatrix, distCoeffs, rvec2, tvec2);
	
	
	
	Mat normal = (Mat_<double>(3, 1) << 0, 0, 1);
	Mat normal1 = rvec1 * normal;

	Mat origin(3, 1, CV_64F, Scalar(0));
	Mat origin1 = rvec1 * origin + tvec1;
	double d_inv1 = 1.0 / normal1.dot(origin1);


	return d_inv1;
}

void Traslation(Sat* Sat1,vector<Mat> ts_decomp, double d_inv,int best) {
	//Finds the xyz translation in then world coordinates
	double d = 1.0 / d_inv;
	Mat ts_abs = d * ts_decomp[best].t();

	//Opdates the POS of the satelite in world frame.
	Sat1->CpOpj.x = ts_abs.at<float>(0, 0);
	Sat1->CpOpj.y = ts_abs.at<float>(1, 0);
	Sat1->CpOpj.z = ts_abs.at<float>(2, 0);
}

int GetFrame(string Filename, Mat* Outframe) {
	// Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
	VideoCapture cap(Filename);
	Mat frame;
	// Check if camera opened successfully
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	while (1) {
		
		
		// Capture frame-by-frame
		cap >> frame;

		// If the frame is empty, break immediately
		if (frame.empty())
			break;

		// Display the resulting frame
		imshow("Frame", frame);

		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	destroyAllWindows();

	*Outframe = frame;
	return 0;
}