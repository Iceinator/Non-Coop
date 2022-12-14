// Non_Coop.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// opencv_helloworld.cpp : Defines the entry point for the console 
//application.
#include "Header.h"
#include "Functions.h"
int main()
{
	
	/*
	Mat output = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_4\\Cropped\\image_data_1_35.png");
	//Mat output = imread("..\\..\\.\\Data\\314364512_678782477205020_5925544091503977094_n.jpg");
	Mat img2 = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_4\\Cropped\\image_data_1_36.png");
	//Mat img2 = imread("..\\..\\.\\Data\\314379773_814713596451058_951117416348934086_n.jpg");
	//resize(output, output, Size(854, 480));
	//resize(img2, img2, Size(854, 480));
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
		imshow("Image 1", output);
		imshow("Image 2", img2);
		imshow("Matched keypoints", img_match);
		imshow("Keypoints img1", keypointimg1);
		imshow("Keypoints img2", keypointimg2);
		char c = (char)waitKey(10);
		if (c == 27) break; //Press escape to stop program 
	}

	vector<Point2f> matched_keypoints1, matched_keypoints2; // these are your points that match



	Mat homography, res;

	vector<KeyPoint> matched_1;
	vector<KeyPoint> matched_2;

	homographyCalculator(&matched_keypoints, &keypoints1, &keypoints2, &homography, &output, &img2, &res, &matched_1, &matched_2);
	Mat K = (Mat_<double>(3, 3) << 1084.68897884346, 0, 297.086796634874, 0, 1084.57557605294, 249.571718427411, 0, 0, 1);
	//Mat homography = K * homography_euclid * K.inv();
	
	

	cout << homography;
	vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
	int solutions = decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
	int best_decomp;
	double current_minimum = 500;
	for (int i = 0; i < solutions; i++)
	// Focal length = 5mm
	{
		double factor_d1 = 0.5;
		Mat rvec_decomp;
		Rodrigues(Rs_decomp[i], rvec_decomp);
		cout << "Solution " << i << ":" << endl;
		cout << "rvec from homography decomposition: " << rvec_decomp.t() << endl;
		double sqrt_squares = sqrt(pow(rvec_decomp.at<double>(0, 0), 2) + pow(rvec_decomp.at<double>(1, 0), 2) + pow(rvec_decomp.at<double>(2, 0), 2));
		if (sqrt_squares <= current_minimum) {
			current_minimum = sqrt_squares;
			best_decomp = i;
		}
		cout << "Length of rotation vector: " << sqrt_squares << "\n";
		cout << "Best solution: " << best_decomp << "\n";
		//cout << "rvec from camera displacement: " << rvec_1to2.t() << endl;
		cout << "tvec from homography decomposition: " << ts_decomp[i].t() << " and scaled by d: " << factor_d1 * ts_decomp[i].t() << endl;
		//cout << "tvec from camera displacement: " << t_1to2.t() << endl;
		cout << "plane normal from homography decomposition: " << normals_decomp[i].t() << endl;
		//cout << "plane normal at camera 1 pose: " << normal1.t() << endl << endl;
	}
	// GT Normals = (1,-1,1)
	//best_decomp = 2;
	Vec3f X=(1,0,0);
	X[0] = 1.0;
	Vec3f Y = (0, 0, 0);
	Y[1] = -1.0;
	Vec3f Z = (0, 0, 0);
	Z[2] = 1;
	Vec3f X_R = rotateVector(X, Rs_decomp[best_decomp]);
	Vec3f X_R_Scaled;
	Vec3f Y_R_Scaled;
	Vec3f Z_R_Scaled;
	X_R_Scaled[0] = X_R[0] * 50;
	X_R_Scaled[1] = X_R[1] * 50;
	X_R_Scaled[2] = X_R[2] * 50;
	Vec3f Y_R = rotateVector(Y, Rs_decomp[best_decomp]);
	Y_R_Scaled[0] = Y_R[0] * 50;
	Y_R_Scaled[1] = Y_R[1] * 50;
	Y_R_Scaled[2] = Y_R[2] * 50;
	Vec3f Z_R = rotateVector(Z, Rs_decomp[best_decomp]);
	Z_R_Scaled[0] = Z_R[0] * 50;
	Z_R_Scaled[1] = Z_R[1] * 50;
	Z_R_Scaled[2] = Z_R[2] * 50;
	
	
	Point2f p = Cp(output);
	Point2f p_t = CpTrack(homography, p);


	//Point2f p_t = p + Point2f(ts_decomp[best_decomp].at<double>(0,0), ts_decomp[best_decomp].at<double>(1, 0));

	//Point_<int> cp = Point2i(sumX/Nkeypoint, sumY/Nkeypoint);
	Point_<int> cendx = p_t + Point2f(X_R_Scaled[0], X_R_Scaled[1]);
	Point_<int> cendy = p_t + Point2f(Y_R_Scaled[0], Y_R_Scaled[1]);
	Point_<int> cendz = p_t + Point2f(Z_R_Scaled[0], Z_R_Scaled[1]);
	Point_<int> cex = p + Point2f(50, 0);
	Point_<int> cey = p + Point2f(0, -50);
	Point_<int> cez = p + Point2f(0, 0);

	arrowedLine(img2, p_t, cendx, Scalar(255, 0, 0), 1);
	arrowedLine(img2, p_t, cendy, Scalar(0, 255, 0), 1);
	arrowedLine(img2, p_t, cendz, Scalar(0, 0, 255), 1);
	arrowedLine(output, p, cex, Scalar(255, 0, 0), 1);
	arrowedLine(output, p, cey, Scalar(0, 255, 0), 1);
	arrowedLine(output, p, cez, Scalar(0, 0, 255), 1);
	//cout << keypoints2.pt;
	while (1) {
		imshow("Matched keypoints RANSAC", res);
		imshow("Original axes", output);
		imshow("Attempt at axes", img2);

		char c = (char)waitKey(10);
		if (c == 27) break; //Press escape to stop program 
	}
	*/

	//Does it on a Number of images.



	/*____________________init______________________________*/
	Sat Cubesat;
	Mat descriptor1;
	Mat descriptor2;
	Mat img_match;
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	vector<DMatch> matched_keypoints;
	Mat homography, res, imgRef, skak1, imgObs, imgRefselect;
	vector<Mat> Rs_decomp, ts_decomp, normals_decomp;
	Mat K = (Mat_<double>(3, 3) << 1084.68897884346, 0, 297.086796634874, 0, 1084.57557605294, 249.571718427411, 0, 0, 1);
	/*_______________________________________________________________________________________________________________________*/

	//Taking the ref image, Press ESC to pic the frame
	//int f = GetFrame("..\\..\\.\\Data\\Cam-2\\Rotation.MOV", &imgRefselect);
	//imwrite("..\\..\\.\\Data\\Cam-2\\Ref_image.png", imgRefselect);
	//Loading images
	//string Path0 = "..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_" + to_string(45) + ".png";
	//Mat imgRef = imread(Path0);
	
	//Calc the distance d in the ref image
	//Load the chess boad images
	//f = GetFrame("..\\..\\.\\Data\\Cam-2\\Chess.MOV", &skak1);
	//Mat skak1 = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_");
	//Mat skak2 = imread("..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_");
	

	//Finding the distance to the opject in ref image

	//float SquarSize=0.022; //22mm Squar Size for the used board
	//double d = Distance(skak1, &Cubesat, SquarSize);

	//f = GetFrame("..\\..\\.\\Data\\Cam-2\\TransDown.MOV", &imgObs);
	
	//FindKeypointsRef(imgRef, &Cubesat, 500);
	
	//MatchKeypoints(imgRef, imgObs, &img_match, &Cubesat, &matched_keypoints, 500);
	//homographyCalculator(&matched_keypoints, &Cubesat, &homography, &imgRef, &imgObs, &res);
	//vector<Mat> Rs_decomp, ts_decomp, normals_decomp;

	//int solutions = decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
	//int best = BestRotSolution(solutions, Rs_decomp, ts_decomp, normals_decomp, d);
	//Cubesat.CenterPoint = CpTrack(homography, Cp0);
	//Cubesat.Rotation.x = float(ts_decomp[best]);

	//cout << "\nRs comp values" << Rs_decomp[best];
	//DrawPOS(&imgObs, Rs_decomp, best, &Cubesat);

	//imshow("End", imgObs);
	//imshow("Ref", imgRef);
	//char c = (char)waitKey(10);
	
	//Loop the hole video





	imgRef = imread("..\\..\\.\\Data\\Cam-2\\Ref_image.png");
	if (!imgRef.empty()) {
		cout << "\nFailed to load image" << endl;
	}
	Point2f Cp0 = Cp(imgRef);
	FindKeypointsRef(imgRef, &Cubesat, 500);
	double d = -172.45;
	VideoCapture cap("..\\..\\.\\Data\\Cam-2\\Rotation.MOV");
	Mat frame;
	if (!cap.isOpened()) {
		cout << "Error opening video stream or file" << endl;
		
	}
	char r;
	while (1) {


		// Capture frame-by-frame
		cap >> imgObs;
		if (imgObs.empty()) {
			cout << "\n No frame";
			break;


		}
		r = (char)waitKey(25);
		if (r == 13) {
			MatchKeypoints(imgRef, imgObs, &img_match, &Cubesat, &matched_keypoints, 500);
			homographyCalculator(&matched_keypoints, &Cubesat, &homography, &imgRef, &imgObs, &res);
			//Cubesat.CenterPoint = CpTrack(homography, Cp0);
			int solutions = decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
			int best = BestRotSolution(solutions, Rs_decomp, ts_decomp, normals_decomp, d);
			Cubesat.CenterPoint = CpTrack(homography, Cp0);
			//Cubesat.Rotation.x = float(ts_decomp[best]);


			DrawPOS(&imgObs, Rs_decomp, best, &Cubesat);

			//Update ref dicripeter and keypoints
			//Cubesat.descriptor1 = Cubesat.descriptor2;
			//Cubesat.keypoints1 = Cubesat.keypoints2;


		}
		
		// If the frame is empty, break immediately
	

		// Display the resulting frame
		//imshow("Frame", frame);

		imshow("End", imgObs);
		// Press  ESC on keyboard to exit
		char c = (char)waitKey(25);
		if (c == 27)
			break;
		imgRef = imgObs;
	}

	// When everything done, release the video capture object
	cap.release();

	// Closes all the frames
	destroyAllWindows();

	/*
	//Mat descriptor1;
	for (int i = 46; i < 92;i++) {
		
		string Path2 = "..\\..\\.\\Data\\image_analysis_data\\Serie_8\\Cropped\\image_data_2_" + to_string(i+1) + ".png";
		Mat imgObs = imread(Path2);
	
		MatchKeypoints(imgRef, imgObs, &img_match,&Cubesat, &matched_keypoints, 500);
		homographyCalculator(&matched_keypoints,&Cubesat, &homography, &imgRef, &imgObs, &res);
		vector<Mat> Rs_decomp, ts_decomp, normals_decomp;

		int solutions = decomposeHomographyMat(homography, K, Rs_decomp, ts_decomp, normals_decomp);
		int best = BestRotSolution(solutions, Rs_decomp, ts_decomp, normals_decomp,d);
		Cubesat.CenterPoint = CpTrack(homography, Cp0);
		//Cubesat.Rotation.x = float(ts_decomp[best]);

		cout << "\nRs comp values" << Rs_decomp[best];
		DrawPOS(&imgObs, Rs_decomp, best, &Cubesat);

		string Rotation[3] = { "x:","y:","z:" };
		//Rotation[0] = Rotation[0] + to_string(ts_decomp[0]);
		//Rotation[1] = Rotation[1] + to_string(ts_decomp[1]);
		//Rotation[2] = Rotation[2] + to_string(ts_decomp[2]);

		putText(imgObs, Rotation[0], Point(10, imgObs.rows / 2+30), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 255), 2);
		putText(imgObs, Rotation[1], Point(10, imgObs.rows / 2), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 2);
		putText(imgObs, Rotation[2], Point(10, imgObs.rows / 2-30), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
		imshow("End", imgObs);
		char c = (char)waitKey(10);


	}
	*/
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
