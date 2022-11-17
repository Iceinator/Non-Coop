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
int homographyCalculator(vector<DMatch>* matched_keypoints, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* homography, Mat* img1, Mat* img2, Mat* res,double nn_match_ratio = 1000,double ransac_thresh = 2.5f) {
	vector<KeyPoint> matched1, matched2;
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
	Mat inlier_mask, homographytemp;
	vector<KeyPoint> inliers1, inliers2;
	vector<DMatch> inlier_matches;
	if (matched1.size() >= 4) {
		homographytemp = findHomography(Points(matched1), Points(matched2),
			RANSAC, ransac_thresh, inlier_mask);
	}
	else {
		cout << "Error: Homography determination failed";
		return 1;
	}
	//cout << homography;

	for (unsigned i = 0; i < matched1.size(); i++) {
		if (inlier_mask.at<uchar>(i)) {
			int new_i = static_cast<int>(inliers1.size());
			inliers1.push_back(matched1[i]);
			inliers2.push_back(matched2[i]);
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