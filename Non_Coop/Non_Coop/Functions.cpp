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

Mat MatchKeypoints(Mat img_1, Mat img_2, int n_features = 50, vector<KeyPoint> keypoints_1,) {
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
	return img_match, keypoints_1, keypoints_2, descriptor_1, descriptor_2;
}