#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

const std::string FILE_DIR = "E:/MOET Work/text_mapping/data/calibration_images/";
const std::string FILE_ADDRESS = FILE_DIR + "FileAddress.txt";

std::vector<std::string> LoadConfig(std::string FileAddress, cv::Size* pBoardSizeOut, cv::Size* pBoardSizeMmOut )
{
	std::ifstream LOADFILE(FileAddress.c_str());
	if (!LOADFILE.is_open())
		std::cout <<"File Failed to Open\n";
	else
		std::cout <<"File opened!\n";

	std::string firstLine;
	if ( getline(LOADFILE, firstLine) )
	{
		std::stringstream ss( firstLine );
		ss >> pBoardSizeOut->width >> pBoardSizeOut->height >> pBoardSizeMmOut->width >> pBoardSizeMmOut->height;
	}

	std::string value;
	std::vector<std::string> Output;

	while (getline(LOADFILE, value))
	{
		
		Output.push_back(value.c_str());
	}


	return Output;

}

int main(int argc, char** argv)
{
	std::cout << "Starting stereo Calibration" << std::endl;

	cv::Size boardSize; 
	cv::Size boardSizeMm;

	std::vector<std::string> NameLocation = LoadConfig(FILE_ADDRESS,&boardSize,&boardSizeMm);

	float squareWidth = ((float)boardSizeMm.width/1000.0f)/(boardSize.width-1);
	float squareHeight = ((float)boardSizeMm.height/1000.0f)/(boardSize.height-1);

	//The points positions in pixels
	std::vector<std::vector<cv::Point2f>> imagePoints;

	for(int i = 0; i < NameLocation.size(); i++)
	{
		std::string ImageAddress = NameLocation.at(i);
		std::cout << "Image Name " <<ImageAddress << std::endl;
		cv::Mat image,colorimage1,colorimage2;
		image =cv::imread(FILE_DIR + ImageAddress, CV_LOAD_IMAGE_GRAYSCALE);

		// output vector of image points
		std::vector<cv::Point2f> imageCorners;

		// number of corners on the chessboard
		bool found = cv::findChessboardCorners(image,boardSize,imageCorners);
		imagePoints.push_back(imageCorners);
		std::cout << "Points detected = " <<imageCorners.size() << std::endl;

	}

	cv::Mat fundemental = cv::findFundamentalMat(cv::Mat(imagePoints.at(0)),cv::Mat(imagePoints.at(1)),CV_FM_RANSAC);

	std::cout << fundemental << std::endl;




	std::cout << "Finished stereo Calibration" << std::endl;
	std::cin.ignore();
	return 0;
}

