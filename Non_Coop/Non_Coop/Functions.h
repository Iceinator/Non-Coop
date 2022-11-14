#pragma once
Mat rgbtogray(Mat input_img);
Mat CannyThreshold(Mat src_gray);
void MatchKeypoints(Mat img_1, Mat img_2, Mat* out_img, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* descriptor1, Mat* descriptor2, int n_features = 50);