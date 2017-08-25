// CornerDetector.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include "CornerDetector.h"

#define INPUT_IMG "test.jpg"

vector<CornerPoint> result;

int _tmain(int argc, _TCHAR* argv[])
{
	cv::Mat src;

#ifdef INPUT_IMG
	src = imread( INPUT_IMG, 0 );
	resize(src,src,Size(IMAGE_W,IMAGE_H),0,0,CV_INTER_LINEAR);
#endif

	unsigned char *ImageY;
	ImageY = (unsigned char  *) malloc(sizeof(unsigned char  ) * IMAGE_W * IMAGE_H); 

	int index = 0;
	for(int i=0; i<IMAGE_H ;i++) {
		for (int j=0; j<IMAGE_W ;j++){
			index = i * IMAGE_W + j;
			ImageY[index] = (unsigned char)src.at<uchar>(i,j);
		}
	}

	cv::Mat showy(IMAGE_H, IMAGE_W, CV_8UC1);
	unsigned char *ptr1 = showy.data;
	int idx=0;
	for(int i=0; i<IMAGE_H; i++){
		for(int j=0; j<IMAGE_W; j++){
			ptr1[idx] = ImageY[idx];
			idx++;
		}
	}
	imshow("showy",showy);
	cvWaitKey(1);

#if 1
	FAST_t(ImageY,40,true);

	for(int i=0;i<result.size();i++){
		//printf("(%lf,%lf)\n",result[i].x,result[i].y);
		circle(src, Point((int)result[i].x,(int)result[i].y), 5, Scalar(0,255,0));
	}
	//printf("size : %d",result.size());
#endif

#if 0
	// vector of keyPoints  
    std::vector<KeyPoint> keyPoints;  
    // construction of the fast feature detector object  
    FastFeatureDetector fast(40,false);
    // feature point detection  
    fast.detect(src,keyPoints);  
    drawKeypoints(src, keyPoints, src, Scalar::all(-1), DrawMatchesFlags::DRAW_OVER_OUTIMG);

#endif

	imshow("Result_src", src);
	cvWaitKey(1);

	system("pause");
	return 0;
}

void FAST_t(unsigned char  *img, int threshold, bool nonmax_suppression)
{
	const int K = patternSize/2, N = patternSize + K + 1;

	int i, j, k, pixel[25];
	makeOffsets(pixel, IMAGE_W, patternSize);

	threshold = (threshold>255) ? 255 : (threshold<0) ? 0 : threshold;

	unsigned char threshold_tab[512];
    for( i = -255; i <= 255; i++ )
        threshold_tab[i+255] = (unsigned char)(i < -threshold ? 1 : i > threshold ? 2 : 0);

	/*AutoBuffer<uchar> _buf((IMAGE_W+16)*3*(sizeof(int) + sizeof(unsigned char)) + 128);
	unsigned char* buf[3];
    buf[0] = _buf; buf[1] = buf[0] + IMAGE_W; buf[2] = buf[1] + IMAGE_W;
    int* cpbuf[3];
    cpbuf[0] = (int*)alignPtr(buf[2] + IMAGE_W, sizeof(int)) + 1;
    cpbuf[1] = cpbuf[0] + IMAGE_W + 1;
    cpbuf[2] = cpbuf[1] + IMAGE_W + 1;
    memset(buf[0], 0, IMAGE_W*3);*/
	
	unsigned char *_buf = (unsigned char  *) malloc(sizeof(unsigned char  ) * (IMAGE_W+16)*3 );
	unsigned char *buf[3];
    buf[0] = _buf; buf[1] = buf[0] + IMAGE_W; buf[2] = buf[1] + IMAGE_W;

	int *cpbuf_tmp = (int  *) malloc(sizeof(int  ) * (IMAGE_W+1)*3 ); 
	int* cpbuf[3];
	cpbuf[0] = cpbuf_tmp;
    cpbuf[1] = cpbuf[0] + IMAGE_W + 1;
    cpbuf[2] = cpbuf[1] + IMAGE_W + 1;
	memset(buf[0], 0, IMAGE_W*3);

	for(i = 3; i < IMAGE_H-2; i++)
    {
		unsigned char* ptr = img + (i * IMAGE_W) + 3;
		unsigned char* curr = buf[(i - 3)%3];
        int* cornerpos = cpbuf[(i - 3)%3];
        memset(curr, 0, IMAGE_W);
        int ncorners = 0;

		if( i < IMAGE_H - 3 )
        {
			j = 3;
			for( ; j < IMAGE_W - 3; j++, ptr++ )
            {
				int v = ptr[0];
				const unsigned char* tab = &threshold_tab[0] - v + 255;
				int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if( d & 1 )
                {
                    int vt = v - threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x < vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (unsigned char)cornerScore(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }

                if( d & 2 )
                {
                    int vt = v + threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x > vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (unsigned char)cornerScore(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }
			}
		}

		cornerpos[-1] = ncorners;

        if( i == 3 )
            continue;

        const uchar* prev = buf[(i - 4 + 3)%3];
        const uchar* pprev = buf[(i - 5 + 3)%3];
        cornerpos = cpbuf[(i - 4 + 3)%3];
        ncorners = cornerpos[-1];

		for( k = 0; k < ncorners; k++ )
        {
            j = cornerpos[k];
            int score = prev[j];
            if( !nonmax_suppression ||
               (score > prev[j+1] && score > prev[j-1] &&
                score > pprev[j-1] && score > pprev[j] && score > pprev[j+1] &&
                score > curr[j-1] && score > curr[j] && score > curr[j+1]) )
            {
				CornerPoint temp;
				temp.x = j;
				temp.y = i-1;
				temp.score = score;
				result.push_back(temp);
            }
        }

	}

}

int Corner_min(int a, int b)
{
	return (a>b) ? b : a;
}

//corner 16
int cornerScore(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

    int a0 = threshold;
    for( k = 0; k < 16; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        a = std::min(a, (int)d[k+3]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+4]);
        a = std::min(a, (int)d[k+5]);
        a = std::min(a, (int)d[k+6]);
        a = std::min(a, (int)d[k+7]);
        a = std::min(a, (int)d[k+8]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+9]));
    }

    int b0 = -a0;
    for( k = 0; k < 16; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        b = std::max(b, (int)d[k+4]);
        b = std::max(b, (int)d[k+5]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+6]);
        b = std::max(b, (int)d[k+7]);
        b = std::max(b, (int)d[k+8]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+9]));
    }

    threshold = -b0-1;

    return threshold;
}

void makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int (*offsets)[2] = patternSize == 16 ? offsets16 :
                              patternSize == 12 ? offsets12 :
                              patternSize == 8  ? offsets8  : 0;

    int k = 0;
    for( ; k < patternSize; k++ )
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for( ; k < 25; k++ )
        pixel[k] = pixel[k - patternSize];
}