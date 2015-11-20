#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H
#include <opencv2\core.hpp>

class CameraCalibrator
{
public:
	CameraCalibrator();
	~CameraCalibrator();
	//Run calibrator process with your camera
	bool runCalibrateProcess(const cv::Size &boardSize, const int &camID);

	//Run calibrator process with a number of pictures
	bool runCalibrateProcess(const cv::Size &boardSize, const std::vector<std::string> &FileList);

	//Save the matrix and resolution ratio to an xml file
	bool SaveResult(const std::string &savePath);

	//Load the xml file for undisorting images
	bool loadXMLFile(const std::string &inputPath);

	//Undisort the image
	bool undisortImage(const cv::Mat inputImg, cv::Mat &outputImg);
private:
	cv::Mat map1, map2;
	int countOfSuccess;
	int cameraID;
	cv::Size imgSize;
	std::vector<std::vector<cv::Point3f>> objectPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	int flag;
	double calibrate(cv::Size &imageSize);
	int addChessboradPoints(const std::vector<std::string> &FileList, const cv::Size &boardSize);
	int addChessboradPoints(const cv::Size &boardSize, const int maxFrames);
	void addChessboardPointsSingleFrame(const cv::Size &boardSize, const cv::Mat &input);
	void addPoints(const std::vector<cv::Point2f> &imageCorners, const std::vector<cv::Point3f> &objectCorners);
};

#endif