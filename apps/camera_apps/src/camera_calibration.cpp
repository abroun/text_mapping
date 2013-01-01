#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "text_mapping/utilities.h"
#include <stdexcept>

//--------------------------------------------------------------------------------------------------
std::vector<std::string> LoadConfig(std::string fileAddress,
    cv::Size* pBoardSizeOut, cv::Size* pBoardSizeMmOut,
    std::string* pCameraNameOut, bool* pbUseDotPatternOut )
{
    cv::FileStorage fileStorage( fileAddress, cv::FileStorage::READ );

    if ( !fileStorage.isOpened() )
    {
        throw std::runtime_error( "Unable to open config file" );
    }

    // Read out BoardSize
    cv::FileNode boardSizeNode = fileStorage[ "BoardSize" ];
    if ( boardSizeNode.size() != 2 || !boardSizeNode[ 0 ].isInt() || !boardSizeNode[ 1 ].isInt() )
    {
        throw std::runtime_error( "Unable to read BoardSize" );
    }
    pBoardSizeOut->width = (int)(boardSizeNode[ 0 ]);
    pBoardSizeOut->height = (int)(boardSizeNode[ 1 ]);

    // Read out BoardSizeMM
    cv::FileNode boardSizeMMNode = fileStorage[ "BoardSizeMM" ];
    if ( boardSizeMMNode.size() != 2 || !boardSizeMMNode[ 0 ].isInt() || !boardSizeMMNode[ 1 ].isInt() )
    {
        throw std::runtime_error( "Unable to read BoardSizeMM" );
    }
    pBoardSizeMmOut->width = (int)(boardSizeMMNode[ 0 ]);
    pBoardSizeMmOut->height = (int)(boardSizeMMNode[ 1 ]);

    // Read out CameraName
    cv::FileNode cameraNameNode = fileStorage[ "CameraName" ];
    if ( !cameraNameNode.isString() )
    {
        throw std::runtime_error( "Unable to read CameraName" );
    }
    *pCameraNameOut = (std::string)cameraNameNode;

    // Read out UseDotPattern
    *pbUseDotPatternOut = false;
    cv::FileNode useDotPatternNode = fileStorage[ "UseDotPattern" ];
    if ( useDotPatternNode.isInt() )
    {
        *pbUseDotPatternOut = ((int)useDotPatternNode != 0);
    }

    // Read in the image file names
    std::vector<std::string> imageFilenames;

    cv::FileNode imageFilesNode = fileStorage[ "ImageFiles" ];
    for ( uint32_t fileIdx = 0; fileIdx < imageFilesNode.size(); fileIdx++ )
    {
        imageFilenames.push_back( Utilities::decodeRelativeFilename(
            fileAddress, (std::string)(imageFilesNode[ fileIdx ]) ) );
    }

    fileStorage.release();

	return imageFilenames;
}

//--------------------------------------------------------------------------------------------------
void showUsage( const char* programName )
{
    printf( "%s configFilename\n", programName );
    printf( "\tconfigFilename - The configuration file giving the calibration images and chessboard size\n" );
}

//--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // Read in a configuration file
    if ( argc < 2 )
    {
        fprintf( stderr, "No configuration file provided\n" );
        showUsage( argv[ 0 ] );
        return -1;
    }

	std::cout << "Starting Camera Calibration" << std::endl;

	cv::Size boardSize; 
	cv::Size boardSizeMm;

	cv::Size imageSize;

	std::string cameraName;
	bool bUseDotPattern;

	std::string configFilename( argv[ 1 ] );
	std::vector<std::string> NameLocation = LoadConfig(
	    configFilename,&boardSize,&boardSizeMm,&cameraName,&bUseDotPattern);

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

	for(int i = 0; i < (int)NameLocation.size(); i++)
	{
		std::string ImageAddress = NameLocation.at(i);
		std::cout << "Image Name " <<ImageAddress << std::endl;
		cv::Mat image,colorimage1,colorimage2;
		image =cv::imread(ImageAddress, CV_LOAD_IMAGE_GRAYSCALE);
		//colorimage1 =cv::imread(ImageAddress, CV_LOAD_IMAGE_COLOR);
		//colorimage2 =cv::imread(ImageAddress, CV_LOAD_IMAGE_COLOR);

		imageSize.height = image.rows;
		imageSize.width = image.cols;

		/*cv::imshow("Image",image);
		cv::waitKey();
		cv::destroyWindow("Image");*/

		// output vector of image points
		std::vector<cv::Point2f> imageCorners;

		// number of corners on the chessboard
		bool found = false;
		if ( !bUseDotPattern )
		{
		    found = cv::findChessboardCorners(image,boardSize,imageCorners);

		    if ( found )
		    {
                //cv::drawChessboardCorners(colorimage1,boardSize,imageCorners,found);

                //Get subpixel accuracy on the corners
                //cv::cornerSubPix(image,imageCorners,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,100,0.225));

                //cv::drawChessboardCorners(colorimage2,boardSize,imageCorners,found);
		    }
		}
		else
		{
		    printf( "Looking for circles...\n" );
		    found = cv::findCirclesGrid(image,boardSize,imageCorners);
		}

		if ( !found )
		{
		    printf( "Warning: Unable to find corners in %s\n", ImageAddress.c_str() );
		    continue;
		}

		//cv::drawChessboardCorners(colorimage1,boardSize,imageCorners,found);


		//If we have a good board, add it to our data
		if(imageCorners.size() == (uint32_t)boardSize.area())
		{
			//Add Image and scene points from one view
			imagePoints.push_back(imageCorners);
			objectPoints.push_back(objectCorners);
			std::cout << "Successfully found " << imageCorners.size() << " corners" << std::endl;
			successes++;
		}
		else
			std::cout << "Failed" << std::endl;

		//cv::resize(colorimage1,colorimage1,cv::Size(1024,768));
		//cv::resize(colorimage2,colorimage2,cv::Size(1024,768));
		//cv::imshow("Pre Sub Pixel",colorimage1);
		//cv::imshow("Post Sub Pixel",colorimage2);
		//cv::waitKey();
		//cv::destroyWindow("Pre Sub Pixel");
		//cv::destroyWindow("Post Sub Pixel");

	}

	std::vector<cv::Mat> rvecs,tvecs;

	distCoeffs = cv::Mat::zeros( 8, 1, CV_64F );

	std::cout << cv::calibrateCamera(
	    objectPoints,imagePoints,imageSize,cameraMatrix,distCoeffs,rvecs,tvecs,
	        CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3
	        | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6 )<< std::endl;

	cv::FileStorage fs((cameraName + "_cameraMatrix.yml").c_str(), cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;

	std::cout << cameraMatrix << std::endl;

	std::cout << "Finished Camera Calibration" << std::endl;

	return 0;
}
