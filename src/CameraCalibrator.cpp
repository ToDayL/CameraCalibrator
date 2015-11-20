#include "stdafx.h"
#include "CameraCalibrator.h"
#include <iostream>
#include <opencv2\videoio.hpp>
#include <opencv2\calib3d.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using cv::VideoCapture;
using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::getTickCount;
using cv::getTickFrequency;
using cv::imread;
using cv::FileStorage;

CameraCalibrator::CameraCalibrator()
{
	flag = 0;
	countOfSuccess = 0;
	objectPoints.resize(0);
	imagePoints.resize(0);
}


CameraCalibrator::~CameraCalibrator()
{
	map1.release();
	map2.release();
	cameraMatrix.release();
	distCoeffs.release();
	objectPoints.resize(0);
	imagePoints.resize(0);
}

int CameraCalibrator::addChessboradPoints(const std::vector<std::string> &FileList, const cv::Size &boardSize)
{
	imagePoints.resize(0);
	objectPoints.resize(0);

	Mat img;
	img = imread(FileList[0]);
	

	imgSize.width = img.cols;
	imgSize.height = img.rows;

	countOfSuccess = 0;
	vector<string>::const_iterator fileListit = FileList.begin();

	for (; fileListit != FileList.end(); fileListit++)
	{

		img = imread((*fileListit));
		cv::cvtColor(img, img, CV_BGR2GRAY);
		addChessboardPointsSingleFrame(boardSize, img);
	}
	return countOfSuccess;
}


void CameraCalibrator::addChessboardPointsSingleFrame(const cv::Size &boardSize, const cv::Mat &input)
{
	vector<Point3f> objectCorners;
	vector<Point2f> imageCorners;
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			objectCorners.push_back(Point3f(i, j, 0.0f));
		}
	}

	imageCorners.resize(0);
	bool found = cv::findChessboardCorners(input, boardSize, imageCorners);

	if (!found)
	{
		return;
	}
	cv::cornerSubPix(input, imageCorners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));

	if (imageCorners.size() == boardSize.area())
	{
		addPoints(imageCorners, objectCorners);
		countOfSuccess++;
	}
}

int CameraCalibrator::addChessboradPoints(const cv::Size &boardSize, const int maxFrames)
{
	imagePoints.resize(0);
	objectPoints.resize(0);
	Mat img;
	VideoCapture cap(cameraID);
	if (!cap.isOpened())
	{
		return 0;
	}

	imgSize.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	imgSize.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	countOfSuccess = 0;
	int64 start = getTickCount();
	for (;;)
	{
		
		cap >> img;
		cv::cvtColor(img, img, CV_BGR2GRAY);
		cv::imshow("Video", img);
		if (cv::waitKey(10) == 27)
		{
			cap.release();
			return countOfSuccess;
		}
		if ((getTickCount() - start) / getTickFrequency() < 2.0)
		{
			continue;
		}

		start = getTickCount();

		addChessboardPointsSingleFrame(boardSize, img);
		if (countOfSuccess == maxFrames)
		{
			cap.release();
			return maxFrames;
		}
	}
}


void CameraCalibrator::addPoints(const std::vector<cv::Point2f> &imageCorners, const std::vector<cv::Point3f> &objectCorners)
{
	imagePoints.push_back(imageCorners);
	objectPoints.push_back(objectCorners);
}


double CameraCalibrator::calibrate(cv::Size &imageSize)
{
	vector<Mat> rvecs, tvecs;

	return cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);
}

bool CameraCalibrator::runCalibrateProcess(const cv::Size &boardSize, const int &camID)
{
	cameraID = camID;
	if (!addChessboradPoints(boardSize, 25))
	{
		return false;
	}
	double err=calibrate(imgSize);
	//The following three lines is designed to show the err, camera matrix and distCoeffs.
	//cout << err << endl;
	//cout << cameraMatrix << endl;
	//cout << distCoeffs << endl;
	VideoCapture cap(camID);
	Mat image;
	while (1)
	{
		cap >> image;
		cv::imshow("window", image);
		Mat map1, map2;
		cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), image.size(), CV_32FC1, map1, map2);
		Mat result;
		cv::remap(image, result, map1, map2, cv::INTER_LINEAR);
		cv::imshow("remap", result);
		if (cv::waitKey(30) == 27)
		{
			break;
		}
	}
	return true;
}


bool CameraCalibrator::runCalibrateProcess(const cv::Size &boardSize, const std::vector<std::string> &FileList)
{
	if (!addChessboradPoints(FileList,boardSize))
	{
		return false;
	}
	double err = calibrate(imgSize);
	//The following three lines is designed to show the err, camera matrix and distCoeffs.
	//cout << err << endl;
	//cout << cameraMatrix << endl;
	//cout << distCoeffs << endl;
	return true;
}


bool CameraCalibrator::SaveResult(const std::string &savePath)
{
	cv::FileStorage fs;
	fs.open(savePath, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		return false;
	}
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs << "imgSize" << imgSize;
	return true;
}

bool CameraCalibrator::loadXMLFile(const std::string &inputPath)
{
	FileStorage fs;
	fs.open(inputPath, FileStorage::READ);
	if (!fs.isOpened())
	{
		return false;
	}
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	fs["imgSize"] >> imgSize;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), imgSize, CV_32FC1, map1, map2);
	return true;
}


bool CameraCalibrator::undisortImage(const cv::Mat inputImg, cv::Mat &outputImg)
{
	if (imgSize.height != inputImg.rows || imgSize.width != inputImg.cols)
	{
		return false;
	}
	cv::remap(inputImg, outputImg, map1, map2, cv::INTER_LINEAR);
	return true;
}