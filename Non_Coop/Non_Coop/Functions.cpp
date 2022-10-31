#include "Header.h"
Mat rgbtogray(Mat input_img) {
	Mat greyMat;
	cvtColor(input_img, greyMat, COLOR_BGR2GRAY);
	return greyMat;
}

Mat CannyThreshold(Mat src_img){
	Mat src;
	src_img.copyTo(src);
	Mat dst, detected_edges;
	uint lowThreshold = 5;
	uint ratio = 5;
	uint kernel_size = 5;
	dst.create(src.size(), src.type());
    blur(src_img, detected_edges, Size(3, 3));
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    dst = Scalar::all(0);
	src.copyTo(dst, detected_edges);
	return dst;
}