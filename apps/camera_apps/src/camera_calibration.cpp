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
	std::cout << "Starting Camera Calibration" << std::endl;

	cv::Size boardSize; 
	cv::Size boardSizeMm;

	cv::Size imageSize;

	std::vector<std::string> NameLocation = LoadConfig(FILE_ADDRESS,&boardSize,&boardSizeMm);

	float squareWidth = ((float)boardSizeMm.width/1000.0f)/(boardSize.width-1);
	float squareHeight = ((float)boardSizeMm.height/1000.0f)/(boardSize.height-1);

	std::cout << squareWidth << "    " << squareHeight << std::endl;

	std::vector<cv::Point3f> objectCorners;

	//The points in the world coordinates 
	std::vector<std::vector<cv::Point3f>> objectPoints;

	//The points positions in pixels
	std::vector<std::vector<cv::Point2f>> imagePoints;

	//Camera output matrices
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	int successes = 0;


	//3D Scene Points
	//Initialize the chessboard corners
	//in the chessboard reference frame
	//The corners are at 3D location (x,y,z) = (i,j,0)
	for(int i=0; i<boardSize.height; i++)
		for(int j=0; j<boardSize.width; j++)
			objectCorners.push_back(cv::Point3f(i*squareHeight,j*squareWidth,0.0f));

	for(int i = 0; i < NameLocation.size(); i++)
	{
		std::string ImageAddress = NameLocation.at(i);
		std::cout << "Image Name " <<ImageAddress << std::endl;
		cv::Mat image,colorimage;
		image =cv::imread(FILE_DIR + ImageAddress, CV_LOAD_IMAGE_GRAYSCALE);
		colorimage =cv::imread(FILE_DIR + ImageAddress, CV_LOAD_IMAGE_COLOR);

		imageSize.height = image.rows;
		imageSize.width = image.cols;

		/*cv::imshow("Image",image);
		cv::waitKey();
		cv::destroyWindow("Image");*/

		// output vector of image points
		std::vector<cv::Point2f> imageCorners;
		// number of corners on the chessboard
		bool found = cv::findChessboardCorners(image,boardSize,imageCorners);

		cv::drawChessboardCorners(colorimage,boardSize,imageCorners,found); 

		//Get subpixel accuracy on the corners
		//cv::cornerSubPix(image,imageCorners,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));

		//If we have a good board, add it to our data
		if(imageCorners.size() == boardSize.area())
		{
			//Add Image and scene points from one view
			imagePoints.push_back(imageCorners);
			objectPoints.push_back(objectCorners);

			successes++;
		}
		else
			std::cout << "Failed" << std::endl;

		cv::imshow("Image",colorimage);
		cv::waitKey();
		cv::destroyWindow("Image");

	}

	std::vector<cv::Mat> rvecs,tvecs;

	std::cout << cv::calibrateCamera(objectPoints,imagePoints,imageSize,cameraMatrix,distCoeffs,rvecs,tvecs,0) << "\n\n" << cameraMatrix << std::endl;




	std::cout << "Finished Camera Calibration" << std::endl;
	std::cin.ignore();
	return 0;
}