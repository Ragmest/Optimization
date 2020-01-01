#pragma once
#include <opencv2/opencv.hpp>
#define PI 3.14159265

class HandRecognition
{
	HandRecognition();
	virtual ~HandRecognition();
	cv::Mat kern = (cv::Mat_<char>(3, 3) << 0,	1, 0,
											1, -4, 1,
											0,	1, 0);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(8, 8), cv::Point(0, 0));
	cv::Mat src, outline, profile, fingerProfile;
	std::vector<cv::Point> outlineVector, fingersOutlinesVector[5];//, sectionsEnds;
	std::vector<cv::Point3i> distancesFromBorder, pointsWithDistancesFromCentroid;
	std::vector<double> distancesFromCentroid, attributes;
	std::vector<std::pair<cv::Point3i, int>> maximumFiveFingerPoints, minimumEncircledFiveFingerPoints;
	//std::list<std::pair<cv::Point3i, int>> minimumList, maximumList;
	cv::Point centroid;
	int blurSize, size;

	void ClearAllVectors();
	cv::Point RotatePoint(const cv::Point& point, const int& angle, const cv::Point& orgin);
	void FindPointsOfOutline();
	void FindFirstPointOfHandOutline();
	void FindSecondPointOfHandOutline();
	void FindHandProfile();
	void FindCentroidByErosion();
	bool checkPoint(const int &x, const int &y, std::vector<cv::Point> &outline);
	void FindDistancesFromCentroid();
	//void MakeListsOfLocalMinAndMax();
	void FindMaximumOfFiveFingersByDividing();
	void FindMinimumBetweenFingers();
	void ExtendMinimumPointsByTwoEncircled();
	void FindFingersOutlines();
	void CalculateAtributes(const std::string& path, const std::string& filename, const bool& isDebug);
	cv::Point3i CheckDistanceFromPoint(const cv::Mat& profile, cv::Point& point);
	void DrawDistRec(cv::Mat& img, cv::Point3i& point);
	void DrawCrossInPoint(cv::Mat& img, cv::Point3i& point, const int& color, const bool& blackDot);
	void DrawLineInPoint(cv::Mat & img, cv::Point & point, const int& color);
	void NormalizeAttributes();
public:
	HandRecognition(HandRecognition const&) = delete;
	void operator=(HandRecognition const&) = delete;
	static HandRecognition& Instance();
	std::vector<double> CalculateAttributesFromHand(const bool& isDebug, const std::string& path, const std::string& filename);
};