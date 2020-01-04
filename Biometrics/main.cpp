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
	std::ofstream out("../optimalization.txt");
	std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
	std::vector<double> answers = HandRecognition::Instance().CalculateAttributesFromHand(true, "../JaJakub", "JaJakub1.jpg");
	std::cout << "----" << "\n";
	std::for_each(answers.begin(), answers.end(), [](double i) { std::cout << i << "\n"; });
	return 0;
}
