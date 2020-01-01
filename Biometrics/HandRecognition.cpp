#include "HandRecognition.h"
#include <opencv2/opencv.hpp>

void SaveImage(const std::string &path, const std::string &name, const cv::Mat& img, const bool& isDebug)
{
	if (isDebug)
		cv::imwrite(path + "/../debug/" + name, img);
}

HandRecognition::HandRecognition()
{
}

HandRecognition::~HandRecognition()
{
}

HandRecognition & HandRecognition::Instance()
{
	static HandRecognition instance;
	return instance;
}

void HandRecognition::FindPointsOfOutline()
{
	FindFirstPointOfHandOutline();
	FindSecondPointOfHandOutline();

	cv::Point current = outlineVector.at(outlineVector.size() - 1);

	while (current.y != (src.rows - 1))
	{
		bool found = false;
		int r = current.x;
		int c = current.y;
		for (int dis = 1; ; dis++)
		{
			int end = dis / 2 + 1;
			for (int x = dis, y = 0; y < end; x--, y++)
			{
				if (y == 0)
				{
					if (r + x < src.cols	&& src.at<uchar>(cv::Point(r + x, c + 0)) == 255)
					{
						found = checkPoint(r + x, c + 0, outlineVector);
						if (found)
							break;
					}
					if (r - x > 0 && src.at<uchar>(cv::Point(r - x, c + 0)) == 255)
					{
						found = checkPoint(r - x, c + 0, outlineVector);
						if (found)
							break;
					}
					if (c + x < src.rows	&& src.at<uchar>(cv::Point(r + 0, c + x)) == 255)
					{
						found = checkPoint(r + 0, c + x, outlineVector);
						if (found)
							break;
					}
					if (c - x > 0 && src.at<uchar>(cv::Point(r + 0, c - x)) == 255)
					{
						found = checkPoint(r + 0, c - x, outlineVector);
						if (found)
							break;
					}
				}
				else
				{
					if (y == x)
					{
						if (r + x < src.cols	&& c + x < src.rows && src.at<uchar>(cv::Point(r + x, c + x)) == 255)
						{
							found = checkPoint(r + x, c + x, outlineVector);
							if (found)
								break;
						}
						if (r - x > 0 && c + x < src.rows && src.at<uchar>(cv::Point(r - x, c + x)) == 255)
						{
							found = checkPoint(r - x, c + x, outlineVector);
							if (found)
								break;
						}
						if (r + x < src.cols	&& c - x > 0 && src.at<uchar>(cv::Point(r + x, c - x)) == 255)
						{
							found = checkPoint(r + x, c - x, outlineVector);
							if (found)
								break;
						}
						if (r - x > 0 && c - x > 0 && src.at<uchar>(cv::Point(r - x, c - x)) == 255)
						{
							found = checkPoint(r - x, c - x, outlineVector);
							if (found)
								break;
						}
					}
					else
					{
						if (r + x < src.cols	&& c + y < src.rows && src.at<uchar>(cv::Point(r + x, c + y)) == 255)
						{
							found = checkPoint(r + x, c + y, outlineVector);
							if (found)
								break;
						}
						if (r + x < src.cols	&& c - y > 0 && src.at<uchar>(cv::Point(r + x, c - y)) == 255)
						{
							found = checkPoint(r + x, c - y, outlineVector);
							if (found)
								break;
						}
						if (r - x > 0 && c + y < src.rows && src.at<uchar>(cv::Point(r - x, c + y)) == 255)
						{
							found = checkPoint(r - x, c + y, outlineVector);
							if (found)
								break;
						}
						if (r - x > 0 && c - y > 0 && src.at<uchar>(cv::Point(r - x, c - y)) == 255)
						{
							found = checkPoint(r - x, c - y, outlineVector);
							if (found)
								break;
						}
						if (r + y < src.cols	&& c + x < src.rows && src.at<uchar>(cv::Point(r + y, c + x)) == 255)
						{
							found = checkPoint(r + y, c + x, outlineVector);
							if (found)
								break;
						}
						if (r + y < src.cols	&& c - x > 0 && src.at<uchar>(cv::Point(r + y, c - x)) == 255)
						{
							found = checkPoint(r + y, c - x, outlineVector);
							if (found)
								break;
						}
						if (r - y > 0 && c + x < src.rows && src.at<uchar>(cv::Point(r - y, c + x)) == 255)
						{
							found = checkPoint(r - y, c + x, outlineVector);
							if (found)
								break;
						}
						if (r - y > 0 && c - x > 0 && src.at<uchar>(cv::Point(r - y, c - x)) == 255)
						{
							found = checkPoint(r - y, c - x, outlineVector);
							if (found)
								break;
						}
					}
				}
			}
			if (found)
				break;
		}

		current = outlineVector.at(outlineVector.size() - 1);
	}
}

cv::Point3i HandRecognition::CheckDistanceFromPoint(const cv::Mat& profile, cv::Point& point)
{
	int r = point.x;
	int c = point.y;
	for (int dis = 1; ; dis++)
	{
		if (c + dis > profile.rows || r + dis > profile.cols || c - dis < 0 || r - dis < 0)
			return cv::Point3i(r, c, dis);
		int end = dis / 2 + 1;
		for (int x = dis, y = 0; y < end; x--, y++)
		{
			if (y == 0)
				if (profile.at<uchar>(cv::Point(r + x, c + 0)) <255 ||
					profile.at<uchar>(cv::Point(r - x, c + 0)) <255 ||
					profile.at<uchar>(cv::Point(r + 0, c + x)) <255 ||
					profile.at<uchar>(cv::Point(r + 0, c - x)) <255)
					return cv::Point3i(r, c, dis);
				else
					if (y == x)
						if (profile.at<uchar>(cv::Point(r + x, c + x)) <255 ||
							profile.at<uchar>(cv::Point(r - x, c + x)) <255 ||
							profile.at<uchar>(cv::Point(r + x, c - x)) <255 ||
							profile.at<uchar>(cv::Point(r - x, c - x)) <255)
							return cv::Point3i(r, c, dis);
						else
							if (profile.at<uchar>(cv::Point(r + x, c + y)) <255 ||
								profile.at<uchar>(cv::Point(r + x, c - y)) <255 ||
								profile.at<uchar>(cv::Point(r - x, c + y)) <255 ||
								profile.at<uchar>(cv::Point(r - x, c - y)) <255 ||
								profile.at<uchar>(cv::Point(r + y, c + x)) <255 ||
								profile.at<uchar>(cv::Point(r + y, c - x)) <255 ||
								profile.at<uchar>(cv::Point(r - y, c + x)) <255 ||
								profile.at<uchar>(cv::Point(r - y, c - x)) <255)
								return cv::Point3i(r, c, dis);
		}
	}
}

void HandRecognition::DrawDistRec(cv::Mat& profile, cv::Point3i& point)
{
	int r = point.x;
	int c = point.y;
	int dis = point.z;
	int end = dis / 2 + 1;
	for (int x = dis, y = 0; y < end; x--, y++)
	{
		if (y == 0)
		{
			profile.at<uchar>(cv::Point(r + x, c + 0)) = 0;
			profile.at<uchar>(cv::Point(r - x, c + 0)) = 0;
			profile.at<uchar>(cv::Point(r + 0, c + x)) = 0;
			profile.at<uchar>(cv::Point(r + 0, c - x)) = 0;
		}
		else
			if (y == x)
			{
				profile.at<uchar>(cv::Point(r + x, c + x)) = 0;
				profile.at<uchar>(cv::Point(r - x, c + x)) = 0;
				profile.at<uchar>(cv::Point(r + x, c - x)) = 0;
				profile.at<uchar>(cv::Point(r - x, c - x)) = 0;
			}
			else
			{
				profile.at<uchar>(cv::Point(r + x, c + y)) = 0;
				profile.at<uchar>(cv::Point(r + x, c - y)) = 0;
				profile.at<uchar>(cv::Point(r - x, c + y)) = 0;
				profile.at<uchar>(cv::Point(r - x, c - y)) = 0;
				profile.at<uchar>(cv::Point(r + y, c + x)) = 0;
				profile.at<uchar>(cv::Point(r + y, c - x)) = 0;
				profile.at<uchar>(cv::Point(r - y, c + x)) = 0;
				profile.at<uchar>(cv::Point(r - y, c - x)) = 0;
			}				
	}
}

void HandRecognition::DrawCrossInPoint(cv::Mat & img, cv::Point3i & point, const int& color, const bool& blackDot)
{
	int r = point.x;
	int c = point.y;
	if (r + 1 < img.rows)
		img.at<uchar>(cv::Point(r + 1, c + 0)) = color;
	if (r - 1 > 0)
		img.at<uchar>(cv::Point(r - 1, c + 0)) = color;
	if (c + 1 < img.cols)
		img.at<uchar>(cv::Point(r + 0, c + 1)) = color;
	if (c - 1  > 0)
		img.at<uchar>(cv::Point(r + 0, c - 1)) = color;
	if (blackDot)
	{
		img.at<uchar>(cv::Point(r, c)) = 0;
		if (r + 2 < img.rows)
			img.at<uchar>(cv::Point(r + 2, c + 0)) = color;
		if (r - 2 > 0)
			img.at<uchar>(cv::Point(r - 2, c + 0)) = color;
		if (c + 2 < img.cols)
			img.at<uchar>(cv::Point(r + 0, c + 2)) = color;
		if (c - 2 > 0)
			img.at<uchar>(cv::Point(r + 0, c - 2)) = color;
	}
	else
		img.at<uchar>(cv::Point(r, c)) = color;
}

void HandRecognition::DrawLineInPoint(cv::Mat & img, cv::Point & point, const int& color)
{
	int r = point.x;
	int c = point.y;
	if (c + 1 < img.cols)
		img.at<uchar>(cv::Point(r + 0, c + 1)) = color;
	if (c - 1 > 0)
		img.at<uchar>(cv::Point(r + 0, c - 1)) = color;
	if (c + 2 < img.cols)
		img.at<uchar>(cv::Point(r + 0, c + 2)) = color;
	if (c - 2 > 0)
		img.at<uchar>(cv::Point(r + 0, c - 2)) = color;
	img.at<uchar>(cv::Point(r, c)) = color;
}

void HandRecognition::NormalizeAttributes()
{
	double distance = attributes[0] * attributes[0];
	std::vector<double>::iterator it;
	for (it = attributes.begin() + 1; it != std::end(attributes); it++)
	{
		distance += *it * *it;
	}
	distance = sqrt(distance);
	double distacneReverted = 1/distance;
	for (it = attributes.begin(); it != std::end(attributes); it++)
	{
		*it = *it * distacneReverted;
	}
}

void HandRecognition::FindFirstPointOfHandOutline()
{
	std::vector<cv::Point3i> firstPointsWithWidth;
	bool found = false;
	for (int i = 0; i < src.cols; i++)
	{
		if (src.at<uchar>(cv::Point(i, src.rows - 1)) == 255)
		{
			if (found == false)
			{
				firstPointsWithWidth.push_back(cv::Point3i(i, src.rows - 1, 0));
				found = true;
			}
			else
			{
				firstPointsWithWidth.back().z = i - firstPointsWithWidth.back().x;
				found = false;
			}
		}
	}
	std::vector<cv::Point3i>::iterator it;
	int max = 0;
	cv::Point point;
	for (it = firstPointsWithWidth.begin(); it != std::end(firstPointsWithWidth); it++)
	{
		if (it->z > max)
		{
			point = cv::Point(it->x, it->y);
			max = it->z;
		}
	}
	outlineVector.push_back(point);
}

void HandRecognition::FindHandProfile()
{
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(0, 0));
	dilate(profile, profile, element); //sometimes floodFill doesn't work on one pixel profile, so we make it wider by dilate
	floodFill(profile, cv::Point(0, 0), cv::Scalar(255));
	cv::bitwise_not(profile, profile);
	erode(profile, profile, element); //here we reversing dilate
}

void HandRecognition::FindSecondPointOfHandOutline()
{
	cv::Point current = outlineVector.back();
	for (int i = -1; i < 2; i++)
	{
		cv::Point p = cv::Point(current.x + i, current.y - 1);
		if (src.at<uchar>(p) == 255)
			outlineVector.push_back(p);
	}
}

bool HandRecognition::checkPoint(const int &x, const int &y, std::vector<cv::Point> &outline)
{
	cv::Point p = cv::Point(x, y);

	std::vector<cv::Point>::reverse_iterator it;
	for (it = outline.rbegin(); it != outline.rend(); ++it)
	{
		if (p.x == it->x && p.y == it->y)
			return false;		
	}

	outline.push_back(p);
	return true;
}

void HandRecognition::FindDistancesFromCentroid()
{
	for (int i = 0; i < outlineVector.size(); i++)
	{
		double x = abs(outlineVector.at(i).x - centroid.x);
		double y = abs(outlineVector.at(i).y - centroid.y);
		double dist = sqrt(x*x + y*y);
		pointsWithDistancesFromCentroid.push_back(cv::Point3i(outlineVector.at(i).x, outlineVector.at(i).y, dist));
		distancesFromCentroid.push_back(dist);
	}
}

/*
void HandRecognition::MakeListsOfLocalMinAndMax()
{
	for (int i = 0; i < pointsWithDistancesFromCentroid.size(); i++)
	{
		if (pointsWithDistancesFromCentroid[i - 1].z < pointsWithDistancesFromCentroid[i].z)
		{
			if (pointsWithDistancesFromCentroid[i + 1].z <= pointsWithDistancesFromCentroid[i].z)
			{
				maximumList.emplace_back(cv::Point3i	(pointsWithDistancesFromCentroid[i].x,
														pointsWithDistancesFromCentroid[i].y,
														pointsWithDistancesFromCentroid[i].z),
														i);
			}
		}
		if (pointsWithDistancesFromCentroid[i - 1].z > pointsWithDistancesFromCentroid[i].z)
		{
			if (pointsWithDistancesFromCentroid[i + 1].z >= pointsWithDistancesFromCentroid[i].z)
			{
				minimumList.emplace_back(cv::Point3i(	pointsWithDistancesFromCentroid[i].x,
														pointsWithDistancesFromCentroid[i].y,
														pointsWithDistancesFromCentroid[i].z),
														i);
			}
		}
	}
}
*/

void HandRecognition::FindMaximumOfFiveFingersByDividing()
{
	int size = pointsWithDistancesFromCentroid.size();
	int section = size * 0.01f; // 1%
	int sectionMin = size * 0.01f; // 1%
	//bool tryOnce = false;
	while (true)
	{
		for (int i = section; i < size; i += section)
		{
			std::pair<cv::Point3i, int> sectionMaximum;// = std::make_pair(cv::Point3i(0, 0, 0), 0);
			for (int j = i - (section * 0.5); j < i + (section * 0.5) && j < size; j++)
			{
				if (pointsWithDistancesFromCentroid[j].z > sectionMaximum.first.z)
				{
					if(maximumFiveFingerPoints.empty() || (maximumFiveFingerPoints.back().second + (section * 0.5)) < j)
						sectionMaximum = std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[j].x,
							pointsWithDistancesFromCentroid[j].y,
							pointsWithDistancesFromCentroid[j].z),
							j);
				}
			}
			if (sectionMaximum.first.z != 0)
				maximumFiveFingerPoints.push_back(sectionMaximum);
			//sectionsEnds.push_back(cv::Point(pointsWithDistancesFromCentroid[i + section].x, pointsWithDistancesFromCentroid[i + section].y));
			//debug
		}
		if (maximumFiveFingerPoints.size() < 5)
		{
			sectionMin = sectionMin * 0.5;
			section = section - sectionMin;
			maximumFiveFingerPoints.clear();
			//sectionsEnds.clear();
		}
		if (maximumFiveFingerPoints.size() == 5)
		{
			break;
		}
		if (maximumFiveFingerPoints.size() > 5)
		{
			section = section + sectionMin;
			maximumFiveFingerPoints.clear();
			//sectionsEnds.clear();
		}
	}
}

void HandRecognition::FindMinimumBetweenFingers()
{
	std::vector<std::pair<cv::Point3i, int>>::iterator it;
	for (it = maximumFiveFingerPoints.begin(); it != std::end(maximumFiveFingerPoints) - 1; it++)
	{
		std::pair<cv::Point3i, int> sectionMinimum = std::make_pair(cv::Point3i(0, 0, INT_MAX), 0);
		for (int i = it->second; i < std::next(it)->second; i++)
		{
			if (sectionMinimum.first.z > pointsWithDistancesFromCentroid[i].z)
			{
				sectionMinimum = std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[i].x,
					pointsWithDistancesFromCentroid[i].y,
					pointsWithDistancesFromCentroid[i].z),
					i);
			}
		}
		minimumEncircledFiveFingerPoints.push_back(sectionMinimum);
	}
}

void HandRecognition::ExtendMinimumPointsByTwoEncircled()
{
	bool found = false;
#define pixelAprox 11

	for (int angle = 1; angle < 90 && found == false; angle++)
	{
		cv::Point rotatedPoint = RotatePoint(cv::Point(minimumEncircledFiveFingerPoints[0].first.x, minimumEncircledFiveFingerPoints[0].first.y), angle, cv::Point(maximumFiveFingerPoints[0].first.x, maximumFiveFingerPoints[0].first.y));
		for (int i = maximumFiveFingerPoints[0].second; i > 0; i--)
		{
			if (abs(pointsWithDistancesFromCentroid[i].y - rotatedPoint.y) < pixelAprox && abs(pointsWithDistancesFromCentroid[i].x - rotatedPoint.x) < pixelAprox)
			{
				minimumEncircledFiveFingerPoints.insert(minimumEncircledFiveFingerPoints.begin(), std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[i].x,
					pointsWithDistancesFromCentroid[i].y,
					pointsWithDistancesFromCentroid[i].z),
					i));
				found = true;
				break;
			}
		}
	}
	if (found == false) 
	{
		//insert first
		minimumEncircledFiveFingerPoints.insert(minimumEncircledFiveFingerPoints.begin(), std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[0].x,
			pointsWithDistancesFromCentroid[0].y,
			pointsWithDistancesFromCentroid[0].z),
			0));
	}
	found = false;
	for (int angle = 0; angle > -90 && found == false; angle--)
	{
		cv::Point rotatedPoint = RotatePoint(cv::Point(minimumEncircledFiveFingerPoints.back().first.x, minimumEncircledFiveFingerPoints.back().first.y), angle, cv::Point(maximumFiveFingerPoints[4].first.x, maximumFiveFingerPoints[4].first.y));
		for (int i = maximumFiveFingerPoints.back().second; i < pointsWithDistancesFromCentroid.size(); i++)
		{
			if (abs(pointsWithDistancesFromCentroid[i].y - rotatedPoint.y) < pixelAprox && abs(pointsWithDistancesFromCentroid[i].x - rotatedPoint.x) < pixelAprox)
			{
				minimumEncircledFiveFingerPoints.push_back(std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[i].x,
					pointsWithDistancesFromCentroid[i].y,
					pointsWithDistancesFromCentroid[i].z),
					i));
				found = true;
				break;
			}
		}
	}
	if (found == false)
	{
		//push last
		int last = pointsWithDistancesFromCentroid.size() - 1;
		minimumEncircledFiveFingerPoints.push_back(std::make_pair(cv::Point3i(pointsWithDistancesFromCentroid[last].x,
			pointsWithDistancesFromCentroid[last].y,
			pointsWithDistancesFromCentroid[last].z),
			last));
	}
}

void HandRecognition::ClearAllVectors()
{
	outlineVector.clear();
	fingersOutlinesVector[0].clear();
	fingersOutlinesVector[1].clear();
	fingersOutlinesVector[2].clear();
	fingersOutlinesVector[3].clear();
	fingersOutlinesVector[4].clear();
	//sectionsEnds.clear();
	distancesFromBorder.clear();
	pointsWithDistancesFromCentroid.clear();
	distancesFromCentroid.clear();
	attributes.clear();
	maximumFiveFingerPoints.clear();
	minimumEncircledFiveFingerPoints.clear();
}

cv::Point HandRecognition::RotatePoint(const cv::Point& point, const int& angle, const cv::Point& orgin)
{
	float s = sin(angle * PI / 180);
	float c = cos(angle * PI / 180);
	float xnew = (((float)point.x - (float)orgin.x) * c) - (((float)point.y - (float)orgin.y) * s) + (float)orgin.x;
	float ynew = (((float)point.x - (float)orgin.x) * s) + (((float)point.y - (float)orgin.y) * c) + (float)orgin.y;
	return cv::Point(xnew, ynew);
}

void HandRecognition::FindFingersOutlines()
{
	for (int i = 0; i < 5; i++)
	{
		for (int j = minimumEncircledFiveFingerPoints[i].second; j < minimumEncircledFiveFingerPoints[i + 1].second + 1; j++)
		{
			fingersOutlinesVector[i].push_back(outlineVector[j]);
		}
	}
}

void HandRecognition::CalculateAtributes(const std::string& path, const std::string& filename, const bool& isDebug)
{
	double x, y;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6), cv::Point(0, 0));

	x = maximumFiveFingerPoints[0].first.x - minimumEncircledFiveFingerPoints[1].first.x;
	y = maximumFiveFingerPoints[0].first.y - minimumEncircledFiveFingerPoints[1].first.y;
	attributes.push_back(sqrt(x * x + y * y));
	for (int i = 1; i < maximumFiveFingerPoints.size() - 1; i++)
	{
		x = minimumEncircledFiveFingerPoints[i].first.x - maximumFiveFingerPoints[i].first.x;
		y = minimumEncircledFiveFingerPoints[i].first.y - maximumFiveFingerPoints[i].first.y;
		attributes.push_back(sqrt(x * x + y * y));
		x = maximumFiveFingerPoints[i].first.x - minimumEncircledFiveFingerPoints[i + 1].first.x;
		y = maximumFiveFingerPoints[i].first.y - minimumEncircledFiveFingerPoints[i + 1].first.y;
		attributes.push_back(sqrt(x * x + y * y));
	}
	x = minimumEncircledFiveFingerPoints[5].first.x - maximumFiveFingerPoints[4].first.x;
	y = minimumEncircledFiveFingerPoints[5].first.y - maximumFiveFingerPoints[4].first.y;
	attributes.push_back(sqrt(x * x + y * y));

	for (int j = 0; j < 5; j++)
	{
		for (int i = 0; i < fingersOutlinesVector[j].size(); i++)
			fingerProfile.at<uchar>(cv::Point(fingersOutlinesVector[j][i].x, fingersOutlinesVector[j][i].y)) = 255;

		cv::Point target = cv::Point(minimumEncircledFiveFingerPoints[j].first.x,
			minimumEncircledFiveFingerPoints[j].first.y);
		cv::Point current = cv::Point(minimumEncircledFiveFingerPoints[j + 1].first.x,
			minimumEncircledFiveFingerPoints[j + 1].first.y);
		line(fingerProfile, target, current, 255);

		//SaveImage(path, filename + "-10-fingerOutline" + std::to_string(j) + ".bmp", fingerProfile, isDebug);
		dilate(fingerProfile, fingerProfile, element); //sometimes floodFill doesn't work on one pixel profile, so we make it wider by dilate
		floodFill(fingerProfile, cv::Point(0, 0), cv::Scalar(255));
		bitwise_not(fingerProfile, fingerProfile);
		erode(fingerProfile, fingerProfile, element); //here we reversing dilate
		bitwise_and(fingerProfile, profile, fingerProfile);
		SaveImage(path, filename + "-10-fingerProfile" + std::to_string(j) + ".bmp", fingerProfile, isDebug);
		attributes.push_back(contourArea(fingersOutlinesVector[j]));
		fingerProfile = cv::Scalar::all(0);
	}
}

void HandRecognition::FindCentroidByErosion()
{
	double m = 0, M = 255;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(0, 0));
	cv::Mat eroded1 = profile.clone();
	cv::Mat eroded2 = profile.clone();

	cv::copyMakeBorder(eroded1, eroded2, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
	while (cv::countNonZero(eroded2) > 0)
	{
		eroded1 = eroded2.clone();
		cv::erode(eroded1, eroded2, element);
		//cv::imshow("erode", eroded2);
		//cv::waitKey(0);
	}
	cv::minMaxLoc(eroded1, &m, &M, NULL, &centroid);
}

std::vector<double> HandRecognition::CalculateAttributesFromHand(const bool& isDebug, const std::string& path,const std::string& filename)
{
	ClearAllVectors();

	src = cv::imread(path + "/" + filename, cv::IMREAD_COLOR);
	outline = cv::Mat(src.rows, src.cols, 0, cv::Scalar::all(0));
	profile = cv::Mat(src.rows, src.cols, 0, cv::Scalar::all(0));
	fingerProfile = cv::Mat(src.rows, src.cols, 0, cv::Scalar::all(0));

	cvtColor(src, src, cv::COLOR_BGR2GRAY);
	SaveImage(path, filename + "-1-cvtColor.bmp", src, isDebug);

	normalize(src, src, 0, 255, cv::NORM_MINMAX);
	SaveImage(path, filename + "-2-normalize.bmp", src, isDebug);

	size = src.rows * src.cols;
	if (size == (576 * 640)) 
		blurSize = 10;  // TODO: make this scale depending on size.
	else
		blurSize = 25; 
	blur(src, src, cv::Size(blurSize, blurSize));
	SaveImage(path, filename + "-3-blur.bmp", src, isDebug);

	normalize(src, src, 0, 255, cv::NORM_MINMAX);
	threshold(src, src, 37, 255, cv::THRESH_BINARY);
	SaveImage(path, filename + "-4-threshold.bmp", src, isDebug);

	erode(src, src, element);
	SaveImage(path, filename + "-5-erode.bmp", src, isDebug);

	//Canny(src, src, 0, 0, 3);
	filter2D(src, src, src.depth(), kern);
	SaveImage(path, filename + "-6-filter2D.bmp", src, isDebug);

	FindPointsOfOutline();

	for (int i = 0; i < outlineVector.size(); i++)
		profile.at<uchar>(cv::Point(outlineVector[i].x, outlineVector[i].y)) = 255;
	SaveImage(path, filename + "-7-handOutline.bmp", profile, isDebug);

	FindHandProfile();
	SaveImage(path, filename + "-8-handProfile.bmp", profile, isDebug);

	FindCentroidByErosion();
	FindDistancesFromCentroid();

	/*
	if (isDebug)
	{
		std::vector<double> xAxis;
		for (double i = 0; i < outlineVector.size(); i++)
			xAxis.push_back(i);
		PlotDrawer::PlotDrawer::draw("x", "y", xAxis, distancesFromCentroid);
	}
	*/

	FindMaximumOfFiveFingersByDividing();
	FindMinimumBetweenFingers();
	ExtendMinimumPointsByTwoEncircled();
	FindFingersOutlines();

	if (isDebug)
	{
		cv::Point3i centr = CheckDistanceFromPoint(profile, centroid);
		DrawDistRec(profile, centr);
		DrawCrossInPoint(profile, centr, 0, false);
		std::vector<std::pair<cv::Point3i, int>>::iterator it;
		for (it = minimumEncircledFiveFingerPoints.begin(); it != std::end(minimumEncircledFiveFingerPoints); it++)
		{
			DrawCrossInPoint(profile, it->first, 150, true);
		}	
		for (it = maximumFiveFingerPoints.begin(); it != std::end(maximumFiveFingerPoints); it++)
		{
			DrawCrossInPoint(profile, it->first, 100, false);
		}
		std::vector<cv::Point>::iterator it2;
		//for (it2 = sectionsEnds.begin(); it2 != std::end(sectionsEnds); it2++)
		//{
		//	DrawLineInPoint(profile, *it2, 200);
		//}
	}

	SaveImage(path, filename + "-9-centroid.bmp", profile, isDebug);
	CalculateAtributes(path, filename, isDebug);
	NormalizeAttributes();

	return attributes;
}


