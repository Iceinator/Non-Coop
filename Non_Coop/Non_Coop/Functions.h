#pragma once
Mat rgbtogray(Mat input_img);
Mat CannyThreshold(Mat src_gray);
void MatchKeypoints(Mat img_1, Mat img_2, Mat* out_img, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* descriptor1, Mat* descriptor2, vector<DMatch>* matches_out, int n_features = 50);
vector<Point2f> Points(vector<KeyPoint> inkeypoints);
//int homographyCalculator(vector<DMatch>* matched_keypoints, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* homography, Mat* img1, Mat* img2, Mat* res, double nn_match_ratio = 1000, double ransac_thresh = 2.5f);
int homographyCalculator(vector<DMatch>* matched_keypoints, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* homography, Mat* img1, Mat* img2, Mat* res, vector<KeyPoint>* matched_1, vector<KeyPoint>* matched_2, double nn_match_ratio = 1000, double ransac_thresh = 2.5f);
Vec3f rotateVector(Vec3f input, Mat RS);
