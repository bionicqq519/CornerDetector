#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\imgproc\imgproc_c.h>
#include <opencv2\opencv.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

#define IMAGE_W 720
#define IMAGE_H 480

const int patternSize = 16;

void FAST_t(unsigned char  *img, int threshold, bool nonmax_suppression);
void makeOffsets(int pixel[25], int rowStride, int patternSize);
int cornerScore(const uchar* ptr, const int pixel[], int threshold);

typedef struct  _CornerPoint
{
	float x;
	float y;
	float score;
	
}CornerPoint;