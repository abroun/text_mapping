#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

const std::string FILE_DIR = "E:/MOET Work/text_mapping/data/calibration_images/";
const std::string FILE_ADDRESS = FILE_DIR + "StereoFileAddress.txt";

std::vector<std::string> LoadConfig(std::string FileAddress, cv::Size* pBoardSizeOut, cv::Size* pBoardSizeMmOut, int* pSetSize)
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
		ss >> pBoardSizeOut->width >> pBoardSizeOut->height >> pBoardSizeMmOut->width >> pBoardSizeMmOut->height >> *pSetSize;
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
	int setSize;

	std::vector<std::string> NameLocation = LoadConfig(FILE_ADDRESS,&boardSize,&boardSizeMm,&setSize);

	float squareWidth = ((float)boardSizeMm.width/1000.0f)/(boardSize.width-1);
	float squareHeight = ((float)boardSizeMm.height/1000.0f)/(boardSize.height-1);

	//The points positions in pixels
	std::vector<std::vector<cv::Point2f>> kinectPoints;
	std::vector<std::vector<cv::Point2f>> canonPoints;
	std::vector<std::vector<cv::Point2f>> imagePoints;

	std::vector<cv::Point3f> objectCorners;

	//The points in the world coordinates 
	std::vector<std::vector<cv::Point3f>> objectPoints;

	for(int i=0; i<boardSize.height; i++)
		for(int j=0; j<boardSize.width; j++)
			objectCorners.push_back(cv::Point3f(i*squareHeight,j*squareWidth,0.0f));

	objectPoints.push_back(objectCorners);
	
	for(int i = 0; i < (int)NameLocation.size(); i++)
	{
		std::string ImageAddress = NameLocation.at(i);
		std::cout << "Image Name " <<ImageAddress << std::endl;
		cv::Mat image;
		image =cv::imread(FILE_DIR + ImageAddress, CV_LOAD_IMAGE_GRAYSCALE);

		// output vector of image points
		std::vector<cv::Point2f> imageCorners;

		// number of corners on the chessboard
		bool found = cv::findChessboardCorners(image,boardSize,imageCorners);
		cv::cornerSubPix(image,imageCorners,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,100,0.225));
		if(imageCorners.size() == (uint32_t)boardSize.area())
		{
			//Add Image and scene points from one view
			if(image.cols > 641)
			{
				for(int j =0; j < (int)imageCorners.size(); j++)
				{
					imageCorners.at(j).x = imageCorners.at(j).x/3.55;
					imageCorners.at(j).y = imageCorners.at(j).y/3.55;
				}		
			}
			if(i >= setSize)
			{
				canonPoints.push_back(imageCorners);
			}
			else
			{
				kinectPoints.push_back(imageCorners);
			}

			//objectPoints.push_back(objectCorners);
			std::cout << "Successfully found " << imageCorners.size() << " corners" << std::endl;
		}
		else
			std::cout << "Failed" << std::endl;
	}


	cv::Mat kinectMatrix;
	cv::Mat kinectCoeffs;
	cv::Mat canonMatrix;
	cv::Mat canonCoeffs;

	cv::FileStorage kinectStream((FILE_DIR +"Kinect_cameraMatrix.yml").c_str(), cv::FileStorage::READ);
	cv::FileStorage canonStream((FILE_DIR +"canon_zoom4_cameraMatrix_2.yml").c_str(), cv::FileStorage::READ);

	kinectStream["cameraMatrix"] >> kinectMatrix;
	kinectStream["distCoeffs"] >> kinectCoeffs;
	canonStream["cameraMatrix"] >> canonMatrix;
	canonStream["distCoeffs"] >> canonCoeffs;
	cv::Mat R,T,E,F;

	canonMatrix.at<double>(0,0) = canonMatrix.at<double>(0,0)/3.55;
	canonMatrix.at<double>(0,2) = canonMatrix.at<double>(0,2)/3.55;
	canonMatrix.at<double>(1,1) = canonMatrix.at<double>(1,1)/3.55;
	canonMatrix.at<double>(1,2) = canonMatrix.at<double>(1,2)/3.55;

	std::cout << kinectMatrix << "\n\n\n" << std::endl;


	std::cout << canonMatrix << "\n\n\n" << std::endl;



	std::cout << "Error  " <<cv::stereoCalibrate(objectPoints,kinectPoints,canonPoints,kinectMatrix,kinectCoeffs,canonMatrix,canonCoeffs,cv::Size(640,480),R,T,E,F) <<std::endl;

	std::cout << "R\n\n\n" << R << "\n\n\n" << std::endl;

	std::cout << "T\n\n\n"<< T << "\n\n\n" << std::endl;

	std::cout << "E\n\n\n"<< E << "\n\n\n" << std::endl;

	std::cout << "F\n\n\n"<< F << "\n\n\n" << std::endl;

	cv::FileStorage fs((FILE_DIR + "stereo_calibration.yml").c_str(), cv::FileStorage::WRITE);

	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;

	std::cout << "Finished stereo Calibration\n Press ENTER" << std::endl;
	std::cin.ignore();
	return 0;
}

