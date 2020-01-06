#pragma once
#include <opencv2/opencv.hpp>
#define PI 3.14159265

class HandRecognition
{
	HandRecognition();
	virtual ~HandRecognition();
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(8, 8), cv::Point(0, 0));
	cv::Mat src;
	std::vector<cv::Point> outlineVector, fingersOutlinesVector[5];
	std::vector<cv::Point3i> pointsWithDistancesFromMiddle;
	std::vector<double> attributes;
	std::vector<std::pair<cv::Point3i, int>> maximumFiveFingerPoints, minimumEncircledFiveFingerPoints;
	cv::Point middlePoint;
	int blurSize, size;

	void ClearAllVectors();
	void FindPointsOfOutline();
	void FindFirstPointOfHandOutline();
	void FindSecondPointOfHandOutline();
	void FindMiddlePoint();
	void FindDistancesMiddlePoint();
	void FindMaximumOfFiveFingersByDividing();
	void FindMinimumBetweenFingers();
	void ExtendMinimumPointsByTwoEncircled();
	void FindFingersOutlines();
	void CalculateAtributes(const std::string& path, const std::string& filename);
	void NormalizeAttributes();
	cv::Point RotatePoint(const cv::Point& point, const int& angle, const cv::Point& orgin);
public:
	HandRecognition(HandRecognition const&) = delete;
	void operator=(HandRecognition const&) = delete;
	static HandRecognition& Instance();
	std::vector<double> CalculateAttributesFromHand(const std::string& path, const std::string& filename);
};