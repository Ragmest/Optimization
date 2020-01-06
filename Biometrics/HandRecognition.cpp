#include "HandRecognition.h"
#include <opencv2/opencv.hpp>

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
	cv::Point previous = outlineVector.at(outlineVector.size() - 2);
	cv::Point next;

	int diffY = current.y - previous.y;
	int diffX = current.x - previous.x;
	if (diffY == 0)
	{
		if (diffX == -1)
		{
			next = cv::Point(previous.x, previous.y - 1);
		}
		if (diffX == 1)
		{
			next = cv::Point(previous.x, previous.y + 1);
		}
	}
	else
	{
		if (diffY == -1)
		{
			if (diffX == 0)
			{
				next = cv::Point(previous.x + 1, previous.y);
			}
			else
			{
				if (diffX == -1)
				{
					next = cv::Point(previous.x, previous.y - 1);
				}
				if (diffX == 1)
				{
					next = cv::Point(previous.x + 1, previous.y);
				}
			}
		}
		if (diffY == 1)
		{
			if (diffX == 0)
			{
				next = cv::Point(previous.x - 1, previous.y);
			}
			else
			{
				if (diffX == -1)
				{
					next = cv::Point(previous.x - 1, previous.y);
				}
				if (diffX == 1)
				{
					next = cv::Point(previous.x, previous.y + 1);
				}
			}
		}
	}
	previous = next;

	while (current.y != (src.rows - 1))
	{
		diffY = current.y - previous.y;
		diffX = current.x - previous.x;
		if (diffY == 0)
		{
			if (diffX == -1)
			{
				next = cv::Point(previous.x, previous.y - 1);
			}
			if (diffX == 1)
			{
				next = cv::Point(previous.x, previous.y + 1);
			}
		}
		else
		{
			if (diffY == -1)
			{
				if (diffX == 0)
				{
					next = cv::Point(previous.x + 1, previous.y);
				}
				else
				{
					if (diffX == -1)
					{
						next = cv::Point(previous.x, previous.y - 1);
					}
					if (diffX == 1)
					{
						next = cv::Point(previous.x + 1, previous.y);
					}
				}
			}
			if (diffY == 1)
			{
				if (diffX == 0)
				{
					next = cv::Point(previous.x - 1, previous.y);
				}
				else
				{
					if (diffX == -1)
					{
						next = cv::Point(previous.x - 1, previous.y);
					}
					if (diffX == 1)
					{
						next = cv::Point(previous.x, previous.y + 1);
					}
				}
			}
		}

		if (src.at<uchar>(previous) == 0 && src.at<uchar>(next) == 255)
		{
			outlineVector.push_back(next);
			cv::Point tmp = current;
			current = next;
			previous = tmp;
		}
		else
		{
			if (src.at<uchar>(previous) == 255 && src.at<uchar>(next) == 0)
			{
				outlineVector.push_back(previous);
				cv::Point tmp = current;
				current = previous;
				previous = tmp;
			}
			else
			{
				previous = next;
			}
		}
	}
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
	for (int i = 0; i < src.cols; i++)
	{
		if (src.at<uchar>(cv::Point(i, src.rows - 1)) == 255)
		{
			outlineVector.push_back(cv::Point(i, src.rows - 1));
			break;
		}
	}
}

void HandRecognition::FindSecondPointOfHandOutline()
{
	cv::Point current = outlineVector.back();
	if (src.at<uchar>(cv::Point(current.x + 1, current.y - 1)) == 255)
	{
		if (src.at<uchar>(cv::Point(current.x, current.y - 1)) == 255)
		{
			outlineVector.push_back(cv::Point(current.x, current.y - 1));
		}
		else
		{
			outlineVector.push_back(cv::Point(current.x + 1, current.y - 1));
		}
	}
}


void HandRecognition::FindDistancesMiddlePoint()
{
	for (int i = 0; i < outlineVector.size(); i++)
	{
		double x = abs(outlineVector.at(i).x - middlePoint.x);
		double y = abs(outlineVector.at(i).y - middlePoint.y);
		double dist = sqrt(x*x + y*y);
		pointsWithDistancesFromMiddle.push_back(cv::Point3i(outlineVector.at(i).x, outlineVector.at(i).y, dist));
	}
}

void HandRecognition::FindMaximumOfFiveFingersByDividing()
{
	int size = pointsWithDistancesFromMiddle.size();
	int section = size * 0.01f; // 1%
	int sectionMin = size * 0.01f; // 1%
	while (true)
	{
		for (int i = section; i < size; i += section)
		{
			std::pair<cv::Point3i, int> sectionMaximum;
			for (int j = i - (section * 0.5); j < i + (section * 0.5) && j < size; j++)
			{
				if (pointsWithDistancesFromMiddle[j].z > sectionMaximum.first.z)
				{
					if(maximumFiveFingerPoints.empty() || (maximumFiveFingerPoints.back().second + (section * 0.5)) < j)
						sectionMaximum = std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[j].x,
							pointsWithDistancesFromMiddle[j].y,
							pointsWithDistancesFromMiddle[j].z),
							j);
				}
			}
			if (sectionMaximum.first.z != 0)
				maximumFiveFingerPoints.push_back(sectionMaximum);
		}
		if (maximumFiveFingerPoints.size() < 5)
		{
			sectionMin = sectionMin * 0.5;
			section = section - sectionMin;
			maximumFiveFingerPoints.clear();
		}
		if (maximumFiveFingerPoints.size() == 5)
			break;
		if (maximumFiveFingerPoints.size() > 5)
		{
			section = section + sectionMin;
			maximumFiveFingerPoints.clear();
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
			if (sectionMinimum.first.z > pointsWithDistancesFromMiddle[i].z)
			{
				sectionMinimum = std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[i].x,
					pointsWithDistancesFromMiddle[i].y,
					pointsWithDistancesFromMiddle[i].z),
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
			if (abs(pointsWithDistancesFromMiddle[i].y - rotatedPoint.y) < pixelAprox && abs(pointsWithDistancesFromMiddle[i].x - rotatedPoint.x) < pixelAprox)
			{
				minimumEncircledFiveFingerPoints.insert(minimumEncircledFiveFingerPoints.begin(), std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[i].x,
					pointsWithDistancesFromMiddle[i].y,
					pointsWithDistancesFromMiddle[i].z),
					i));
				found = true;
				break;
			}
		}
	}
	if (found == false) 
	{
		//insert first
		minimumEncircledFiveFingerPoints.insert(minimumEncircledFiveFingerPoints.begin(), std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[0].x,
			pointsWithDistancesFromMiddle[0].y,
			pointsWithDistancesFromMiddle[0].z),
			0));
	}
	found = false;
	for (int angle = 0; angle > -90 && found == false; angle--)
	{
		cv::Point rotatedPoint = RotatePoint(cv::Point(minimumEncircledFiveFingerPoints.back().first.x, minimumEncircledFiveFingerPoints.back().first.y), angle, cv::Point(maximumFiveFingerPoints[4].first.x, maximumFiveFingerPoints[4].first.y));
		for (int i = maximumFiveFingerPoints.back().second; i < pointsWithDistancesFromMiddle.size(); i++)
		{
			if (abs(pointsWithDistancesFromMiddle[i].y - rotatedPoint.y) < pixelAprox && abs(pointsWithDistancesFromMiddle[i].x - rotatedPoint.x) < pixelAprox)
			{
				minimumEncircledFiveFingerPoints.push_back(std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[i].x,
					pointsWithDistancesFromMiddle[i].y,
					pointsWithDistancesFromMiddle[i].z),
					i));
				found = true;
				break;
			}
		}
	}
	if (found == false)
	{
		//push last
		int last = pointsWithDistancesFromMiddle.size() - 1;
		minimumEncircledFiveFingerPoints.push_back(std::make_pair(cv::Point3i(pointsWithDistancesFromMiddle[last].x,
			pointsWithDistancesFromMiddle[last].y,
			pointsWithDistancesFromMiddle[last].z),
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
	pointsWithDistancesFromMiddle.clear();
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

bool comparePoint(cv::Point i1, cv::Point i2)
{
	return (i1.y < i2.y);
}

void HandRecognition::CalculateAtributes(const std::string& path, const std::string& filename)
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

	for (int j = 0; j < 5; ++j)
	{
		cv::Point start = fingersOutlinesVector[j].front();
		cv::Point end = fingersOutlinesVector[j].back();

		float tmp1 = (start.y - end.y);
		float tmp2 = (start.x - end.x);
		float a = tmp1 / tmp2;
		float b = start.y - a * start.x;

		if (j!=4)
		{
			for (int i = end.x; i > start.x; --i)
			{
				int y = a * i + b;
				fingersOutlinesVector[j].push_back(cv::Point(i, y));
			}
		}
		else
		{
			bool isDoubledPoint = false;
			float invA = 1 / a;
			for (int i = end.y; i > start.y; --i)
			{
				int x = (i - b) * invA;
				fingersOutlinesVector[j].push_back(cv::Point(x, i));
				if (isDoubledPoint == false)
				{
					for (int k = 0; k < fingersOutlinesVector[j].size() - 2; k++)
					{
						if (fingersOutlinesVector[j].back().x == fingersOutlinesVector[j].at(k).x
							&& fingersOutlinesVector[j].back().y == fingersOutlinesVector[j].at(k).y)
						{
							isDoubledPoint = true;
							fingersOutlinesVector[j].erase(fingersOutlinesVector[j].begin(), fingersOutlinesVector[j].begin() + k);
							break;
						}
					}
					if (isDoubledPoint == true)
						break;
				}
			}
		}
		sort(fingersOutlinesVector[j].begin(), fingersOutlinesVector[j].end(), comparePoint);

		double area = 0;
		int min, max, y = fingersOutlinesVector[j].front().y;
		min = max = fingersOutlinesVector[j].front().x;
		for (int i = 0; i < fingersOutlinesVector[j].size(); ++i)
		{
			cv::Point p = fingersOutlinesVector[j].at(i);
			if (y != p.y)
			{
				y = p.y;
				area += max - min + 1;
				min = max = p.x;
			}
			else
			{
				if (p.x > max)
					max = p.x;
				if (p.x < min)
					min = p.x;
			}		
		}
		area += max - min;
		attributes.push_back(area);
	}
}

void HandRecognition::FindMiddlePoint()
{
	double x = 0, y = 0;
	for (int i = 0; i < outlineVector.size() - 1; ++i)
	{
		x += outlineVector.at(i).x;
		y += outlineVector.at(i).y;
	}
	cv::Point end = outlineVector.back();
	cv::Point start = outlineVector.front();
	for (int i = start.x; i < end.x; ++i)
	{
		x += i;
		y += start.y;
	}
	int size = outlineVector.size() + (end.x - start.x);
	middlePoint = cv::Point(x / size, y / size);
}

std::vector<double> HandRecognition::CalculateAttributesFromHand(const std::string& path,const std::string& filename)
{
	ClearAllVectors();

	//1
	auto start = std::chrono::high_resolution_clock::now();
	src = cv::imread(path + "/" + filename, cv::IMREAD_COLOR);
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//2
	start = std::chrono::high_resolution_clock::now();
	cvtColor(src, src, cv::COLOR_BGR2GRAY);
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//3
	start = std::chrono::high_resolution_clock::now();
	normalize(src, src, 0, 255, cv::NORM_MINMAX);
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//4
	start = std::chrono::high_resolution_clock::now();
	size = src.rows * src.cols;
	if (size == (576 * 640)) 
		blurSize = 10;  // TODO: make this scale depending on size.
	else
		blurSize = 25; 
	blur(src, src, cv::Size(blurSize, blurSize));
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//5
	start = std::chrono::high_resolution_clock::now();
	threshold(src, src, 37, 255, cv::THRESH_BINARY);
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//6
	start = std::chrono::high_resolution_clock::now();
	erode(src, src, element);
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//7
	std::cout << "0.0" << "\n";

	//8
	start = std::chrono::high_resolution_clock::now();
	FindPointsOfOutline();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//9
	std::cout << "0.0" << "\n";

	//10
	start = std::chrono::high_resolution_clock::now();
	FindMiddlePoint();
	FindDistancesMiddlePoint();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//11
	start = std::chrono::high_resolution_clock::now();
	FindMaximumOfFiveFingersByDividing();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//12
	start = std::chrono::high_resolution_clock::now();
	FindMinimumBetweenFingers();
	ExtendMinimumPointsByTwoEncircled();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//13
	start = std::chrono::high_resolution_clock::now();
	FindFingersOutlines();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	//14
	start = std::chrono::high_resolution_clock::now();
	CalculateAtributes(path, filename);
	NormalizeAttributes();
	finish = std::chrono::high_resolution_clock::now();
	elapsed = finish - start;
	std::cout << elapsed.count() << "\n";

	return attributes;
}