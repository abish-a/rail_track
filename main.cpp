#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

float getSlope(Vec4i line)
{
	return ((line[3] - line[1]) / (line[2] - line[0]));
}

float getLength(Vec4i line)
{
	return sqrt(((line[3] - line[1]) * (line[3] - line[1])) + ((line[2] - line[0]) * (line[2] - line[0])));
}

Mat DoHough(Mat dst, Mat orig)
{
	int iDmin = 100;
	int iDmax = 250;
	Mat cdst = orig;
	//Mat cdst = Mat(dst.size(), dst.type());
	vector<Vec4i> lines;
	Vec4i aLongestLines[2] = { 0,0 }; // Choosing longest line that represents rail track

	HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 50, 10);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		for (size_t j = 0; j < lines.size(); j++)
		{
			Vec4i l_2 = lines[j];
			float fM2 = getSlope(l_2);
				if (abs(fM2) > 0)
					if (((abs(l[2] - l_2[2]) < iDmax) && (abs(l[0] - l_2[0])) > iDmin))
					{
						if (fM2 > 0)
							if (getLength(l_2) > getLength(aLongestLines[0]))
								aLongestLines[0] = l_2;
						if (fM2 < 0)
							if (getLength(l_2) > getLength(aLongestLines[1]))
								aLongestLines[1] = l_2;
					}
		}
	}
	line(cdst, Point(aLongestLines[0][0], aLongestLines[0][1]), Point(aLongestLines[0][2], aLongestLines[0][3]), Scalar(0, 0, 255), 1, CV_AA);
	line(cdst, Point(aLongestLines[1][0], aLongestLines[1][1]), Point(aLongestLines[1][2], aLongestLines[1][3]), Scalar(0, 0, 255), 1, CV_AA);
	return cdst;
}

Mat DoCrop(Mat image, float fCropFactor)
{
	cv::Rect roi;
	roi.x = 0;
	roi.y = image.rows * (1 - fCropFactor);

	roi.width = image.size().width;
	roi.height = image.size().height * fCropFactor;
	image = image(roi);
	return image;
}

Mat DoSobel(Mat image)
{
	Mat Result;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	/// Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	Sobel(image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);

	/// Gradient Y
	Sobel(image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);

	/// Total Gradient (approximate)
	addWeighted(abs_grad_x, 1, abs_grad_y, 0, 0, Result);
	//cv::Canny(image, Result, 100, 200);
	return Result;
}

void showWindow(char* title, cv::Mat image)
{
	cv::namedWindow(title, CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
	cv::imshow(title, image);					// show windows
}

int main()
{
	Mat imgOriginal;        // input image
	Mat imgGrayscale;       // grayscale of input image
	Mat imgGradient;		// Sobel Gradient
	Mat imgCropped;
	Mat imgBinary;
	Mat imgHough;
	float fCrop = 0.666667;

	imgOriginal = cv::imread("image.jpg");          // open image
	if (imgOriginal.empty())						// if unable to open image
	{
		return(-1);                                 // exit program
	}
	cv::cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY);       // convert to grayscale
	GaussianBlur(imgGrayscale, imgGrayscale, Size(5, 5), 0, 0, BORDER_DEFAULT);
	imgGradient = DoSobel(imgGrayscale);						// Get Sobel Gradient
	imgCropped = DoCrop(imgGradient, fCrop);		// Cropping the image as we dont need the upper part

	threshold(imgCropped, imgBinary, 0, 255, THRESH_BINARY + THRESH_OTSU);
	dilate(imgBinary, imgBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
	erode(imgBinary, imgBinary, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

	//imgHough = DoHough(imgBinary);
	imgHough = DoHough(imgBinary, DoCrop(imgOriginal, fCrop));

	//showWindow("imgGrayscale", imgGrayscale);
	//showWindow("imgGradient", imgGradient);
	//showWindow("imgBinary", imgBinary);
	showWindow("imgHough", imgHough);

	waitKey(0);
}