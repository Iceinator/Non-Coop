// Non_Coop.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// opencv_helloworld.cpp : Defines the entry point for the console 
//application.
#include "Header.h"
#include "Functions.h"
int main()
{
	//Mat output = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_83.png");
	Mat output = imread("..\\..\\.\\Data\\314364512_678782477205020_5925544091503977094_n.jpg");
	//Mat img2 = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_84.png");
	Mat img2 = imread("..\\..\\.\\Data\\314379773_814713596451058_951117416348934086_n.jpg");
	resize(output, output, Size(854, 480));
	resize(img2, img2, Size(854, 480));
	Mat descriptor1;
	Mat descriptor2;
	Mat img_match;
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<DMatch> matched_keypoints;

	MatchKeypoints(output, img2, &img_match, &keypoints1, &keypoints2, &descriptor1, &descriptor2, &matched_keypoints, 500);
	Mat keypointimg1, keypointimg2;
	drawKeypoints(output, keypoints1, keypointimg1);
	drawKeypoints(img2, keypoints2, keypointimg2);
	//imshow("Matches", img_match);
	while (1) {
		imshow("Matched keypoints", img_match);
		imshow("Keypoints img1", keypointimg1);
		imshow("Keypoints img2", keypointimg2);
		char c = (char)waitKey(10);
		if (c == 27) break; //Press escape to stop program 
	}
	Mat homography, res;
	homographyCalculator(&matched_keypoints, &keypoints1, &keypoints2, &homography, &output, &img2, &res);
	cout << homography;
	Mat K = (Mat_<double>(3, 3) << 1084.68897884346, 0, 297.086796634874, 0, 1084.57557605294, 249.571718427411, 0, 0, 1);
	vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
	int solutions = decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
	for (int i = 0; i < solutions; i++)
	{
		double factor_d1 = 1.0 / 1;
		Mat rvec_decomp;
		Rodrigues(Rs_decomp[i], rvec_decomp);
		cout << "Solution " << i << ":" << endl;
		cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
		//cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
		cout << "tvec from homography decomposition: " << ts_decomp[i].t() << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
		//cout << "tvec from camera displacement: " << t_1to2.t() << endl;
		cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << endl;
		//cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
	}
	Vec3f X=(1,0,0);
	X[0] = 1.0;
	Vec3f Y = (0, 0, 0);
	Y[1] = -1.0;
	Vec3f Z = (0, 0, 0);
	Z[2] = 1;
	Vec3f X_R = rotateVector(X, Rs_decomp[0]);
	Vec3f X_R_Scaled;
	Vec3f Y_R_Scaled;
	Vec3f Z_R_Scaled;
	X_R_Scaled[0] = X_R[0] * 50;
	X_R_Scaled[1] = X_R[1] * 50;
	X_R_Scaled[2] = X_R[2] * 50;
	Vec3f Y_R = rotateVector(Y, Rs_decomp[0]);
	Y_R_Scaled[0] = Y_R[0] * 50;
	Y_R_Scaled[1] = Y_R[1] * 50;
	Y_R_Scaled[2] = Y_R[2] * 50;
	Vec3f Z_R = rotateVector(Z, Rs_decomp[0]);
	Z_R_Scaled[0] = Z_R[0] * 50;
	Z_R_Scaled[1] = Z_R[1] * 50;
	Z_R_Scaled[2] = Z_R[2] * 50;
	float sumX = 0;
	float sumY = 0;
	int Nkeypoint = keypoints2.size();
	for (int i = 0; i<Nkeypoint; i++) {
		Point2f p = keypoints2[i].pt;
		sumX += p.x;
		sumY += p.y;
	}
	Point_<int> cp = Point2i(sumX/Nkeypoint, sumY/Nkeypoint);
	Point_<int> cendx = cp + Point2i(X_R_Scaled[0], X_R_Scaled[1]);
	Point_<int> cendy = cp + Point2i(Y_R_Scaled[0], Y_R_Scaled[1]);
	Point_<int> cendz = cp + Point2i(Z_R_Scaled[0], Z_R_Scaled[1]);
	arrowedLine(img2, cp, cendx, Scalar(255, 0, 0), 1);
	arrowedLine(img2, cp, cendy, Scalar(0, 255, 0), 1);
	arrowedLine(img2, cp, cendz, Scalar(0, 0, 255), 1);
	//cout << keypoints2.pt;
	while (1) {
		imshow("Matched keypoints", res);
		imshow("Attempt at axes", img2);
		char c = (char)waitKey(10);
		if (c == 27) break; //Press escape to stop program 
	}
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
