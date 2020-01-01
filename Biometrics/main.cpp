#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <windows.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cstdlib>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "HandRecognition.h"

int main()
{
	std::vector<double> answer = HandRecognition::Instance().CalculateAttributesFromHand(true, "../JaJakub", "JaJakub1.jpg");
	return 0;
}
