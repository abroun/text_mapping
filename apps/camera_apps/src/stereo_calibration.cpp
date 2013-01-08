#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "text_mapping/point_cloud.h"
#include "text_mapping/utilities.h"
#include <stdexcept>

#include "blob_detector.h"
#include "circlesgrid.hpp"

bool findCirclesGridAB( cv::InputArray _image, cv::Size patternSize,
		cv::OutputArray _centers, int flags, const cv::Ptr<cv::FeatureDetector> &blobDetector )
{
    bool isAsymmetricGrid = (flags & cv::CALIB_CB_ASYMMETRIC_GRID) ? true : false;
    bool isSymmetricGrid  = (flags & cv::CALIB_CB_SYMMETRIC_GRID ) ? true : false;
    CV_Assert(isAsymmetricGrid ^ isSymmetricGrid);

    cv::Mat image = _image.getMat();
    std::vector<cv::Point2f> centers;

    std::vector<cv::KeyPoint> keypoints;
    blobDetector->detect(image, keypoints);
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < keypoints.size(); i++)
    {
      points.push_back (keypoints[i].pt);
    }

    if(flags & cv::CALIB_CB_CLUSTERING)
    {
      CirclesGridClusterFinder circlesGridClusterFinder(isAsymmetricGrid);
      circlesGridClusterFinder.findGrid(points, patternSize, centers);
      cv::Mat(centers).copyTo(_centers);
      return !centers.empty();
    }

    CirclesGridFinderParameters parameters;
    parameters.vertexPenalty = -0.6f;
    parameters.vertexGain = 1;
    parameters.existingVertexGain = 10000;
    parameters.edgeGain = 1;
    parameters.edgePenalty = -0.6f;

    if(flags & cv::CALIB_CB_ASYMMETRIC_GRID)
      parameters.gridType = CirclesGridFinderParameters::ASYMMETRIC_GRID;
    if(flags & cv::CALIB_CB_SYMMETRIC_GRID)
      parameters.gridType = CirclesGridFinderParameters::SYMMETRIC_GRID;

    const int attempts = 2;
    const size_t minHomographyPoints = 4;
    cv::Mat H;
    for (int i = 0; i < attempts; i++)
    {
      centers.clear();
      CirclesGridFinder boxFinder(patternSize, points, parameters);
      bool isFound = false;
//#define BE_QUIET 1
#if BE_QUIET
      void* oldCbkData;
      //cv::ErrorCallback oldCbk = redirectError(quiet_error, 0, &oldCbkData);
#endif
      try
      {
        isFound = boxFinder.findHoles();
      }
      catch (cv::Exception)
      {

      }
#if BE_QUIET
      redirectError(oldCbk, oldCbkData);
#endif
      if (isFound)
      {
      	switch(parameters.gridType)
      	{
          case CirclesGridFinderParameters::SYMMETRIC_GRID:
            boxFinder.getHoles(centers);
            break;
          case CirclesGridFinderParameters::ASYMMETRIC_GRID:
	    boxFinder.getAsymmetricHoles(centers);
	    break;
          default:
            CV_Error(CV_StsBadArg, "Unkown pattern type");
      	}

        if (i != 0)
        {
        	cv::Mat orgPointsMat;
        	cv::transform(centers, orgPointsMat, H.inv());
        	cv::convertPointsFromHomogeneous(orgPointsMat, centers);
        }
        cv::Mat(centers).copyTo(_centers);
        return true;
      }

      boxFinder.getHoles(centers);
      if (i != attempts - 1)
      {
        if (centers.size() < minHomographyPoints)
          break;
        H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
      }
    }
    cv::Mat(centers).copyTo(_centers);
    return false;
}

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
        Utilities::makeFilenameAbsoluteFromCWD( fileAddress ), (std::string)firstCameraCalibrationFileNode );

    // Read out SecondCameraCalibrationFile
    cv::FileNode secondCameraCalibrationFileNode = fileStorage[ "SecondCameraCalibrationFile" ];
    if ( !secondCameraCalibrationFileNode.isString() )
    {
        throw std::runtime_error( "Unable to read SecondCameraCalibrationFile" );
    }
    *pSecondCameraCalibrationFilenameOut = Utilities::decodeRelativeFilename(
        Utilities::makeFilenameAbsoluteFromCWD( fileAddress ), (std::string)secondCameraCalibrationFileNode );
        
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
				Utilities::makeFilenameAbsoluteFromCWD( fileAddress ), (std::string)firstFileNode );
            std::string secondFilename = Utilities::decodeRelativeFilename(
				Utilities::makeFilenameAbsoluteFromCWD( fileAddress ), (std::string)secondFileNode );

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

    cv::Mat image;
    PointCloud::Ptr pCloud;

    // Load the image, either from an image file supported by OpenCV, or from a point cloud
    if ( imageFilename.length() >= 4
		&& imageFilename.substr( imageFilename.length() - 4, 4 ) == ".spc" )
    {
    	pCloud = PointCloud::loadPointCloudFromSpcFile( imageFilename );
    	if ( NULL == pCloud )
    	{
    		fprintf( stderr, "Error: Unable to load %s\n", imageFilename.c_str() );
    		return false;
    	}

    	cv::Mat colourImage = pCloud->getImage();
    	cv::cvtColor( colourImage, image, CV_RGB2GRAY );
    }
    else
    {
    	image = cv::imread( imageFilename, CV_LOAD_IMAGE_GRAYSCALE );
    }

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
        BlobDetector::Params params;

        /*//params.minArea = 5.0;
        params.minArea = 5.0;
        //params.maxArea = 200.0;
        params.minCircularity = 0.5;
        params.minDistBetweenBlobs = 1.0;
        params.filterByCircularity = false;
        params.filterByInertia = false;
        params.filterByConvexity = false;*/

        if ( image.cols > 640 )
        {
            params.maxArea = 20000.0;
        }

        //printf( "minDistBetweenBlobs is %f\n", params.min );

        cv::Ptr<cv::FeatureDetector> pDetector = new BlobDetector( params );

        bCornersFound = findCirclesGridAB( image, boardSize, *pImageCornersOut,
            cv::CALIB_CB_SYMMETRIC_GRID, pDetector );

        printf( "Found %lu of %i corners\n", pImageCornersOut->size(), boardSize.area() );

        if ( !bCornersFound )
        {
            cv::Mat scaled;
            cv::resize( image, scaled, cv::Size( 0, 0 ), 0.25, 0.25 );


			bCornersFound = findCirclesGridAB( scaled, boardSize, *pImageCornersOut,
				cv::CALIB_CB_SYMMETRIC_GRID, pDetector );
			std::cout << "Found " << pImageCornersOut->size() << " corners on second try" << std::endl;

			for ( uint32_t i = 0; i < pImageCornersOut->size(); i++ )
			{
				(*pImageCornersOut)[ i ].x *= 4.0;
				(*pImageCornersOut)[ i ].y *= 4.0;
			}
        }

        cv::namedWindow("Corners", CV_WINDOW_NORMAL);
		cv::drawChessboardCorners(image,boardSize,*pImageCornersOut,bCornersFound);
		cv::imshow( "Corners", image );
		cv::waitKey();
    }

    // Check that we've got the correct number of corners
    if( pImageCornersOut->size() != (uint32_t)boardSize.area() )
    {
        bCornersFound = false;
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

	if ( !firstCalibFile.isOpened() )
	{
		fprintf( stderr, "Error: Unable to open %s\n", firstCameraCalibrationFilename.c_str() );
		return -1;
	}

	if ( !secondCalibFile.isOpened() )
	{
		fprintf( stderr, "Error: Unable to open %s\n", secondCameraCalibrationFilename.c_str() );
		return -1;
	}

	firstCalibFile[ "cameraMatrix" ] >> firstCalibMatrix;
	firstCalibFile[ "distCoeffs" ] >> firstDistCoeffs;
	secondCalibFile[ "cameraMatrix" ] >> secondCalibMatrix;
	secondCalibFile[ "distCoeffs" ] >> secondDistCoeffs;
	cv::Mat R,T,E,F;
    R = cv::Mat::zeros( 3, 1, CV_64FC1 );
    T = cv::Mat::zeros( 3, 1, CV_64FC1 );

	std::cout << "Error  "
	    << cv::stereoCalibrate(
	        objectPoints, secondCameraPoints, firstCameraPoints,
	        secondCalibMatrix, secondDistCoeffs,
	        firstCalibMatrix, firstDistCoeffs,
	        cv::Size(0, 0), R, T, E, F,
	        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6),
	        CV_CALIB_FIX_INTRINSIC  ) << std::endl;

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

	/*
	// Try to solve manually with solvePnP
	cv::Mat firstRotVec, firstTransVec;
	cv::Mat secondRotVec, secondTransVec;
	solvePnP( objectPoints[ 0 ], firstCameraPoints[ 0 ], firstCalibMatrix, firstDistCoeffs, firstRotVec, firstTransVec );
	solvePnP( objectPoints[ 0 ], secondCameraPoints[ 0 ], secondCalibMatrix, secondDistCoeffs, secondRotVec, secondTransVec );

	cv::Mat objectInFirstCameraSpaceMtx = cv::Mat::eye( 4, 4, CV_64F );
	cv::Mat objectInSecondCameraSpaceMtx = cv::Mat::eye( 4, 4, CV_64F );


	cv::Mat firstRotMtx;
	cv::Mat secondRotMtx;

	cv::Rodrigues( firstRotVec, firstRotMtx );
	cv::Rodrigues( secondRotVec, secondRotMtx );

	cv::Mat firstRotTarget( objectInFirstCameraSpaceMtx, cv::Rect( 0, 0, 3, 3 ) );
	cv::Mat firstPosTarget( objectInFirstCameraSpaceMtx, cv::Rect( 3, 0, 1, 3 ) );

	firstRotMtx.copyTo( firstRotTarget );
	firstTransVec.copyTo( firstPosTarget );

	cv::Mat secondRotTarget( objectInSecondCameraSpaceMtx, cv::Rect( 0, 0, 3, 3 ) );
	cv::Mat secondPosTarget( objectInSecondCameraSpaceMtx, cv::Rect( 3, 0, 1, 3 ) );

	secondRotMtx.copyTo( secondRotTarget );
	secondTransVec.copyTo( secondPosTarget );

	cv::Mat secondCameraInFirstCameraSpace = objectInFirstCameraSpaceMtx*objectInSecondCameraSpaceMtx.inv();
	std::cout << "firstCalibMatrix" << std::endl;
	std::cout << firstCalibMatrix << std::endl;
	std::cout << "secondCalibMatrix" << std::endl;
	std::cout << secondCalibMatrix << std::endl;
	std::cout << "secondCameraInFirstCameraSpace" << std::endl;
	std::cout << secondCameraInFirstCameraSpace << std::endl;
*/
	return 0;
}

