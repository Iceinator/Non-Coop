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
	uint lowThreshold = 5;
	uint ratio = 1;
	uint kernel_size = 3;
	dst.create(src.size(), src.type());
    blur(src_img, detected_edges, Size(5, 5));
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    dst = Scalar::all(0);
	src.copyTo(dst, detected_edges);
	return dst;
}