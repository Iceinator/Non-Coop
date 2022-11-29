#pragma once
Mat rgbtogray(Mat input_img);
Mat CannyThreshold(Mat src_gray);
void MatchKeypoints(Mat img_1, Mat img_2, Mat* out_img, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* descriptor1, Mat* descriptor2, vector<DMatch>* matches_out, int n_features = 50);
vector<Point2f> Points(vector<KeyPoint> inkeypoints);
int homographyCalculator(vector<DMatch>* matched_keypoints, vector<KeyPoint>* keypoints1, vector<KeyPoint>* keypoints2, Mat* homography, Mat* img1, Mat* img2, Mat* res, double nn_match_ratio = 1000, double ransac_thresh = 2.5f);
Vec3f rotateVector(Vec3f input, Mat RS);
Point2f CpTrack(Mat H, Point2f Cp);
Point2f Cp(Mat output);
int BestRotSolution(int solutions, vector<Mat> Rs_decomp, vector<Mat>ts_decomp, vector<Mat>normals_decomp);
void DrawPOS(Mat* img2, vector<Mat> Rs_decomp, int best_decomp, Sat* Sat1);
double Distance(Mat Skak1, Mat Skak2, Sat* Sat1);
vector<Point3f> calsChessboardCorners(Size boardSize, int squareSize);
void Traslation(Sat* Sat1, vector<Mat> ts_decomp, double d_inv, int best);