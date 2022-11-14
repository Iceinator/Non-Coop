// Non_Coop.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// opencv_helloworld.cpp : Defines the entry point for the console 
//application.
#include "Header.h"
#include "Functions.h"
int main()
{
	Mat output = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_46.png");
	Mat img2 = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_47.png");
	//resize(output, output, Size(854, 480));
	//resize(img2, img2, Size(854, 480));
	Mat descriptor1;
	Mat descriptor2;
	Mat img_match;
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<DMatch> matched_keypoints;
	
	MatchKeypoints(output,img2,&img_match,&keypoints1,&keypoints2,&descriptor1,&descriptor2,&matched_keypoints,100);
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
	Mat homography,res;
	homographyCalculator(&matched_keypoints, &keypoints1, &keypoints2, &homography, &output, &img2, &res);
	cout << homography;
	Mat K = (Mat_<double>(3, 3) << 1, 0, 1, 0, 1, 1, 0, 0, 1);
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

	while (1) {
		imshow("Matched keypoints", res);
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
