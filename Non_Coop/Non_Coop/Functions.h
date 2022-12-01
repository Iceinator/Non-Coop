#pragma once
Mat rgbtogray(Mat input_img);
Mat CannyThreshold(Mat src_gray);
void MatchKeypoints(Mat imgRef, Mat imgObs, Mat* out_img, Sat* Sat1, vector<DMatch>* matches_out, int n_features = 50);
vector<Point2f> Points(vector<KeyPoint> inkeypoints);
int homographyCalculator(vector<DMatch>* matched_keypoints, Sat* Sat1, Mat* homography, Mat* img1, Mat* img2, Mat* res, double nn_match_ratio = 1000, double ransac_thresh = 2.5f);
Vec3f rotateVector(Vec3f input, Mat RS);
Point2f CpTrack(Mat H, Point2f Cp);
Point2f Cp(Mat output);
int BestRotSolution(int solutions, vector<Mat> Rs_decomp, vector<Mat>ts_decomp, vector<Mat>normals_decomp, double d_inv1);
void DrawPOS(Mat* img2, vector<Mat> Rs_decomp, int best_decomp, Sat* Sat1);
double Distance(Mat Skak1, Sat* Sat1, float squareSize);
vector<Point3f> calsChessboardCorners(Size boardSize, int squareSize);
void Traslation(Sat* Sat1, vector<Mat> ts_decomp, double d_inv, int best);
void FindKeypointsRef(Mat Ref, Sat* Sat1, int n_features = 50);
int GetFrame(string Filename, Mat* frame);
//MatchKeypoints(Mat imgRef, Mat imgObs, Mat* out_img, Sat* Sat1, vector<KeyPoint>* keypoints2, Mat* descriptor2, vector<DMatch>* matches_out, int n_features = 50);