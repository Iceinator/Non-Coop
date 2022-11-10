// Non_Coop.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// opencv_helloworld.cpp : Defines the entry point for the console 
//application.
#include "Header.h"
#include "Functions.h"
int main()
{
	Mat output = imread("..\\..\\.\\Data\\313203981_1538771779893678_1321708788536571748_n.jpg");
	Mat img2 = imread("..\\..\\.\\Data\\314379773_814713596451058_951117416348934086_n.jpg");
	resize(output, output, Size(854, 480));
	resize(img2, img2, Size(854, 480));

	//while (1) {
	//	imshow("Loaded image", output);
	//	char c = (char)waitKey(10);
	//	if (c == 27) break; //Press escape to stop program 
	//}
	//Mat grayout = rgbtogray(output);
	//while (1) {
	//imshow("Gray output", grayout);
	//char c = (char)waitKey(10);
	//if (c == 27) break; //Press escape to stop program 
	//}
	Mat descriptor1, descriptor2, img_match;
	vector<KeyPoint> keypoints1, keypoints2;

	img_match, keypoints1, keypoints2, descriptor1, descriptor2 = MatchKeypoints(output, img2, 50);
	Mat keypointimg1, keypointimg2;
	drawKeypoints(output, keypoints1, keypointimg1);
	drawKeypoints(img2, keypoints2, keypointimg2);
	//imshow("Matches", img_match);
	while (1) {
		//imshow("Matched keypoints", img_match);
		imshow("Keypoints img1", keypointimg1);
		imshow("Keypoints img2", keypointimg2);
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
