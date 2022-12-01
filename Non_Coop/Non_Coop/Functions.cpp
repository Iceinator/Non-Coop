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

void MatchKeypoints(Mat img_1, Mat img_2,Mat* out_img, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* descriptor1, Mat* descriptor2,vector<DMatch>* matches_out, int n_features = 50) {
	Mat descriptor_1, descriptor_2;
	vector<KeyPoint> keypoints_1, keypoints_2;
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	Ptr<ORB> detector = ORB::create(n_features);
	Ptr<DescriptorExtractor> extractor;
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	detector->compute(img_1, keypoints_1, descriptor_1);
	detector->compute(img_2, keypoints_2, descriptor_2);
	matcher->match(descriptor_1, descriptor_2, matches);
	Mat img_match;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//return img_match, keypoints_1, keypoints_2, descriptor_1, descriptor_2;
	*out_img = img_match.clone();
	*keypoints1 = keypoints_1;
	*keypoints2 = keypoints_2;
	*descriptor1 = descriptor_1.clone();
	*descriptor2 = descriptor_2.clone();
	*matches_out = matches;
}
vector<Point2f> Points(vector<KeyPoint> inkeypoints) {
	vector<Point2f> out;
	KeyPoint::convert(inkeypoints,out, std::vector< int >());
	return out;
}
int homographyCalculator(vector<DMatch>* matched_keypoints, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* homography, Mat* img1, Mat* img2, Mat* res, vector<KeyPoint>* matched_1, vector<KeyPoint>* matched_2, double nn_match_ratio = 1000,double ransac_thresh = 2.5f) { //
	vector<KeyPoint> matched1 = *matched_1;
	vector<KeyPoint> matched2 = *matched_2;
	vector<DMatch> mkeypoints = *matched_keypoints;
	vector<KeyPoint> kpoints1 = *keypoints1;
	vector<KeyPoint> kpoints2 = *keypoints2;
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
	*matched_1 = matched1_;
	*matched_2 = matched2_;
	return 0;
}

Vec3f rotateVector(Vec3f input, Mat RS) {
	Mat input_t = Mat(input);
	Mat RS_f;
	RS.convertTo(RS_f, CV_32FC1);
	Mat rotated_vector = RS_f * input_t;
	return Vec3f(rotated_vector);
}
