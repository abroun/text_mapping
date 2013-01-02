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
std::vector<std::pair<std::string, std::string> > LoadConfig(std::string fileAddress,
    cv::Size* pBoardSizeOut, cv::Size* pBoardSizeMmOut,
    std::string* pFirstCameraNameOut, std::string* pSecondCameraNameOut,
    std::string* pFirstCameraCalibrationFilenameOut,
    std::string* pSecondCameraCalibrationFilenameOut,
    bool* pbUseDotPatternOut )
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

    // Read out FirstCameraName
    cv::FileNode firstCameraNameNode = fileStorage[ "FirstCameraName" ];
    if ( !firstCameraNameNode.isString() )
    {
        throw std::runtime_error( "Unable to read FirstCameraName" );
    }
    *pFirstCameraNameOut = (std::string)firstCameraNameNode;

    // Read out SecondCameraName
    cv::FileNode secondCameraNameNode = fileStorage[ "SecondCameraName" ];
    if ( !secondCameraNameNode.isString() )
    {
        throw std::runtime_error( "Unable to read SecondCameraName" );
    }
    *pSecondCameraNameOut = (std::string)secondCameraNameNode;

    // Read out UseDotPattern
    *pbUseDotPatternOut = false;
    cv::FileNode useDotPatternNode = fileStorage[ "UseDotPattern" ];
    if ( useDotPatternNode.isInt() )
    {
        *pbUseDotPatternOut = ((int)useDotPatternNode != 0);
    }

    // Read out FirstCameraCalibrationFile
    cv::FileNode firstCameraCalibrationFileNode = fileStorage[ "FirstCameraCalibrationFile" ];
    if ( !firstCameraCalibrationFileNode.isString() )
    {
        throw std::runtime_error( "Unable to read FirstCameraCalibrationFile" );
    }
    *pFirstCameraCalibrationFilenameOut = Utilities::decodeRelativeFilename(
        fileAddress, (std::string)firstCameraCalibrationFileNode );

    // Read out SecondCameraCalibrationFile
    cv::FileNode secondCameraCalibrationFileNode = fileStorage[ "SecondCameraCalibrationFile" ];
    if ( !secondCameraCalibrationFileNode.isString() )
    {
        throw std::runtime_error( "Unable to read SecondCameraCalibrationFile" );
    }
    *pSecondCameraCalibrationFilenameOut = Utilities::decodeRelativeFilename(
        fileAddress, (std::string)secondCameraCalibrationFileNode );

    // Read in the image file names
    std::vector<std::pair<std::string, std::string> > imageFilenamePairs;

    cv::FileNode imageFilePairsNode = fileStorage[ "ImageFilePairs" ];
    for ( uint32_t pairIdx = 0; pairIdx < imageFilePairsNode.size(); pairIdx++ )
    {
        cv::FileNode pairNode = imageFilePairsNode[ pairIdx ];
        cv::FileNode firstFileNode = pairNode[ "First" ];
        cv::FileNode secondFileNode = pairNode[ "Second" ];

        if ( !firstFileNode.isString() || !secondFileNode.isString() )
        {
            printf( "Warning: Unable to read filename pair %i\n", pairIdx );
        }
        else
        {
            std::string firstFilename = Utilities::decodeRelativeFilename(
                fileAddress, (std::string)firstFileNode );
            std::string secondFilename = Utilities::decodeRelativeFilename(
                fileAddress, (std::string)secondFileNode );

            imageFilenamePairs.push_back( std::pair<std::string, std::string>(
                firstFilename, secondFilename ) );
        }
    }

    fileStorage.release();

    return imageFilenamePairs;
}

//--------------------------------------------------------------------------------------------------
void showUsage( const char* programName )
{
    printf( "%s configFilename\n", programName );
    printf( "\tconfigFilename - The configuration file giving the calibration images and chessboard size\n" );
}

//--------------------------------------------------------------------------------------------------
// Locates corners in the image. Returns true if the corners were found and false otherwise
bool findImageCorners( std::string imageFilename, cv::Size boardSize, bool bUseDotPattern,
    std::vector<cv::Point2f>* pImageCornersOut )
{
    bool bCornersFound = false;
    pImageCornersOut->clear();

    cv::Mat image = cv::imread( imageFilename, CV_LOAD_IMAGE_GRAYSCALE );

    if ( !bUseDotPattern )
    {
        // number of corners on the chessboard
        bCornersFound = cv::findChessboardCorners( image, boardSize, *pImageCornersOut );
        if ( bCornersFound )
        {
            cv::cornerSubPix(image,*pImageCornersOut,cv::Size(11,11),
                cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,100,0.225));
        }
    }
    else
    {
        cv::SimpleBlobDetector::Params params;

        //params.minArea = 5.0;
        params.minArea = 5.0;
        //params.maxArea = 200.0;
        params.minCircularity = 0.5;
        params.minDistBetweenBlobs = 1.0;
        params.filterByCircularity = false;
        params.filterByInertia = false;
        params.filterByConvexity = false;
        //printf( "minDistBetweenBlobs is %f\n", params.min );

        cv::Ptr<cv::FeatureDetector> pDetector = new cv::SimpleBlobDetector( params );

        bCornersFound = cv::findCirclesGrid( image, boardSize, *pImageCornersOut,
            cv::CALIB_CB_SYMMETRIC_GRID, pDetector );

        printf( "Found %lu of %i corners\n", pImageCornersOut->size(), boardSize.area() );

        //cv::drawChessboardCorners(image,boardSize,*pImageCornersOut,bCornersFound);
        //cv::imshow( "Corners", image );
        //cv::waitKey();
        //cv::destroyWindow("Corners");
    }

    // Check that we've got the correct number of corners
    if( pImageCornersOut->size() != (uint32_t)boardSize.area() )
    {
        bCornersFound = false;
    }
    else
    {
        for ( uint32_t cornerIdx = 0; cornerIdx < pImageCornersOut->size(); cornerIdx++ )
        {
            (*pImageCornersOut)[ cornerIdx ].x = image.cols - (*pImageCornersOut)[ cornerIdx ].x;
            (*pImageCornersOut)[ cornerIdx ].y = image.rows - (*pImageCornersOut)[ cornerIdx ].y;
        }
    }

    return bCornersFound;
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

	std::cout << "Starting stereo Calibration" << std::endl;

	cv::Size boardSize; 
	cv::Size boardSizeMm;
	std::string firstCameraName;
	std::string secondCameraName;
	std::string firstCameraCalibrationFilename;
	std::string secondCameraCalibrationFilename;
	bool bUseDotPattern;

	std::string configFilename( argv[ 1 ] );
	std::vector<std::pair<std::string,std::string> > imageFilenamePairs = LoadConfig(
	    configFilename, &boardSize, &boardSizeMm, &firstCameraName, &secondCameraName,
	    &firstCameraCalibrationFilename, &secondCameraCalibrationFilename, &bUseDotPattern );

	// Make sure that we have some image pairs to work with
	if ( imageFilenamePairs.size() <= 0 )
	{
	    fprintf( stderr, "Error: No image pairs specified\n" );
	    return -1;
	}




	//The points positions in pixels
	std::vector<std::vector<cv::Point2f>> firstCameraPoints;
	std::vector<std::vector<cv::Point2f>> secondCameraPoints;
	
	for(int i = 0; i < (int)imageFilenamePairs.size(); i++)
	{
	    printf( "Processing image pair %i\n", i );

	    std::vector<cv::Point2f> firstCameraCorners;
	    std::vector<cv::Point2f> secondCameraCorners;

	    bool bFirstCameraCornersFound = findImageCorners(
	        imageFilenamePairs[ i ].first, boardSize, bUseDotPattern, &firstCameraCorners );
	    bool bSecondCameraCornersFound = findImageCorners(
            imageFilenamePairs[ i ].second, boardSize, bUseDotPattern, &secondCameraCorners );

	    if ( !bFirstCameraCornersFound )
	    {
	        printf( "Warning: Unable to find corners in %s\n", imageFilenamePairs[ i ].first.c_str() );
	        continue;
	    }
	    if ( !bSecondCameraCornersFound )
        {
            printf( "Warning: Unable to find corners in %s\n", imageFilenamePairs[ i ].second.c_str() );
            continue;
        }

	    firstCameraPoints.push_back( firstCameraCorners );
	    secondCameraPoints.push_back( secondCameraCorners );
	}

	// Make sure that we have some image pairs to work with
    if ( firstCameraPoints.size() <= 0 )
    {
        fprintf( stderr, "Error: Unable to find enough image points\n" );
        return -1;
    }

    float squareWidth = ((float)boardSizeMm.width/1000.0f)/(boardSize.width-1);
    float squareHeight = ((float)boardSizeMm.height/1000.0f)/(boardSize.height-1);
    std::vector<cv::Point3f> objectCorners;

    //The points in the world coordinates
    std::vector<std::vector<cv::Point3f>> objectPoints;

    for(int i=boardSize.height-1; i>=0; i--)
    //for(int i=0; i<boardSize.height; i++)
        for(int j=0; j<boardSize.width; j++)
            objectCorners.push_back(cv::Point3f(j*squareWidth,i*squareHeight,0.0f));

    for ( uint32_t pairIdx = 0; pairIdx < firstCameraPoints.size(); pairIdx++ )
    {
        objectPoints.push_back(objectCorners);
    }


	cv::Mat firstCalibMatrix;
	cv::Mat firstDistCoeffs;
	cv::Mat secondCalibMatrix;
	cv::Mat secondDistCoeffs;

	cv::FileStorage firstCalibFile( firstCameraCalibrationFilename, cv::FileStorage::READ );
	cv::FileStorage secondCalibFile( secondCameraCalibrationFilename, cv::FileStorage::READ );

	firstCalibFile[ "cameraMatrix" ] >> firstCalibMatrix;
	firstCalibFile[ "distCoeffs" ] >> firstDistCoeffs;
	secondCalibFile[ "cameraMatrix" ] >> secondCalibMatrix;
	secondCalibFile[ "distCoeffs" ] >> secondDistCoeffs;
	cv::Mat R,T,E,F;

	std::cout << "Error  "
	    << cv::stereoCalibrate(
	        objectPoints, secondCameraPoints, firstCameraPoints,
	        secondCalibMatrix, secondDistCoeffs,
	        firstCalibMatrix, firstDistCoeffs,
	        cv::Size(0, 0), R, T, E, F ) << std::endl;

	std::cout << "R\n\n\n" << R << "\n\n\n" << std::endl;

	std::cout << "T\n\n\n"<< T << "\n\n\n" << std::endl;

	std::cout << "E\n\n\n"<< E << "\n\n\n" << std::endl;

	std::cout << "F\n\n\n"<< F << "\n\n\n" << std::endl;

	cv::FileStorage fs( firstCameraName + "_to_" + secondCameraName + "_calib.yml", cv::FileStorage::WRITE);

	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;

	std::cout << "Finished stereo Calibration" << std::endl;

	return 0;
}

