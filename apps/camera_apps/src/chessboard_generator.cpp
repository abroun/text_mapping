/*

Copyright (c) 2012, Bristol Robotics Laboratory
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Bristol Robotics Laboratory nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE BRISTOL ROBOTICS LABORATORY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//--------------------------------------------------------------------------------------------------
// File: chessboard_generator.cpp
// Desc: Reads in a camera configuration and generates synthetic images of a chessboard from a
//       number of viewpoints. These images can be used to check the quality of the camera
//       calibration pipeline.
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "text_mapping/point_cloud.h"
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
struct PoseData
{
    PoseData( const cv::Vec3d& position, const cv::Vec3d& rotXYZ )
        : position( position ), rotXYZ( rotXYZ ) {}

    cv::Vec3d position;
    cv::Vec3d rotXYZ;       // Used to specify orientation assuming that normal starts as +ve z-axis
};

//--------------------------------------------------------------------------------------------------
std::vector<PoseData> gHighResCalibrationPositions;
std::vector<PoseData> gRelativeCalibrationPositions;

enum ePixelColour
{
	ePC_None = -1,
	ePC_Black = 0,
	ePC_White
};

// Represents a point in an RGB or depth image
struct PointData
{
    int32_t mPixelX;
    int32_t mPixelY;
    double mWorldX;
    double mWorldY;
    double mWorldZ;

    ePixelColour mPixelColour;
};

// TODO: Move these to the config file
const double CHESSBOARD_SQUARE_SIDE_LENGTH = 0.02;
const double CHESSBOARD_BORDER_WIDTH = 0.04;
const int32_t CHESSBOARD_WIDTH = 9;
const int32_t CHESSBOARD_HEIGHT = 7;
const bool CHESSBOARD_TOP_LEFT_CORNER_IS_BLACK = true;

const double CHESSBOARD_TOTAL_WIDTH = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_WIDTH*CHESSBOARD_SQUARE_SIDE_LENGTH;
const double CHESSBOARD_TOTAL_HEIGHT = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_HEIGHT*CHESSBOARD_SQUARE_SIDE_LENGTH;

bool gbDrawCircles = true;

const double CIRCLEBOARD_SQUARE_SIDE_LENGTH = 0.02;
const double CIRCLEBOARD_BORDER_WIDTH = 0.04;
const int32_t CIRCLEBOARD_WIDTH = 8;
const int32_t CIRCLEBOARD_HEIGHT = 6;

const double CIRCLEBOARD_TOTAL_WIDTH = 2.0*CIRCLEBOARD_BORDER_WIDTH + (CIRCLEBOARD_WIDTH-1)*CIRCLEBOARD_SQUARE_SIDE_LENGTH;
const double CIRCLEBOARD_TOTAL_HEIGHT = 2.0*CIRCLEBOARD_BORDER_WIDTH + (CIRCLEBOARD_HEIGHT-1)*CIRCLEBOARD_SQUARE_SIDE_LENGTH;
const double CIRCLEBOARD_CIRCLE_RADIUS = 0.005;

//--------------------------------------------------------------------------------------------------
void showUsage( const char* programName )
{
    printf( "%s configFilename\n", programName );
    printf( "\tconfigFilename - The name of the configuration file for the camera rig\n" );
}

//--------------------------------------------------------------------------------------------------
void initialiseTestPositions()
{
    // Set up positions so that the high resolution camera can be calibrated
    gHighResCalibrationPositions.clear();

    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), 0.0 ) ) );

    // Rotate around X
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
            cv::Vec3d( 0.0, Utilities::degToRad( 140.0 ), 0.0 ) ) );
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 220.0 ), 0.0 ) ) );

    // Rotate around Y
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( Utilities::degToRad( -40.0 ), Utilities::degToRad( 180.0 ), 0.0 ) ) );
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( Utilities::degToRad( 40.0 ), Utilities::degToRad( 180.0 ), 0.0 ) ) );

    // Rotate around Z
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), Utilities::degToRad( -40.0 ) ) ) );
    gHighResCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), Utilities::degToRad( 40.0 ) ) ) );

    // Set up positions so that the relative camera poses can be found
    gRelativeCalibrationPositions.clear();

    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), 0.0 ) ) );

    // Rotate around X
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.7 ),
            cv::Vec3d( 0.0, Utilities::degToRad( 140.0 ), 0.0 ) ) );
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.9 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 220.0 ), 0.0 ) ) );

    // Rotate around Y
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( Utilities::degToRad( -40.0 ), Utilities::degToRad( 180.0 ), 0.0 ) ) );
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.7 ),
        cv::Vec3d( Utilities::degToRad( 40.0 ), Utilities::degToRad( 180.0 ), 0.0 ) ) );

    // Rotate around Z
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.8 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), Utilities::degToRad( -40.0 ) ) ) );
    gRelativeCalibrationPositions.push_back(
        PoseData( cv::Vec3d( 0.0, 0.1, 0.9 ),
        cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), Utilities::degToRad( 40.0 ) ) ) );
}

//--------------------------------------------------------------------------------------------------
std::string createOutputFilename( const char* rootName, uint32_t number, const char* extension )
{
    std::ostringstream buf;
    buf << rootName << "_" << number << extension;
    return buf.str();
}

//--------------------------------------------------------------------------------------------------
cv::Mat createCameraCalibrationMatrix( double focalLengthPixel, int32_t imageWidth, int32_t imageHeight )
{
    cv::Mat calibMtx = cv::Mat::eye( 3, 3, CV_64FC1 );

    double principleX = (double)imageWidth/2.0;
    double principleY = (double)imageHeight/2.0;

    calibMtx.at<double>( 0, 0 ) = focalLengthPixel;
    calibMtx.at<double>( 1, 1 ) = focalLengthPixel;
    calibMtx.at<double>( 0, 2 ) = principleX;
    calibMtx.at<double>( 1, 2 ) = principleY;

    return calibMtx;
}

//--------------------------------------------------------------------------------------------------
cv::Mat createRotationMatrixX( double angle )
{
    double cosAngle = cos( angle );
    double sinAngle = sin( angle );

    cv::Mat rotMtxX = cv::Mat::eye( 3, 3, CV_64FC1 );
    rotMtxX.at<double>( 1, 1 ) = cosAngle;
    rotMtxX.at<double>( 1, 2 ) = -sinAngle;
    rotMtxX.at<double>( 2, 1 ) = sinAngle;
    rotMtxX.at<double>( 2, 2 ) = cosAngle;

    return rotMtxX;
}

//--------------------------------------------------------------------------------------------------
cv::Mat createRotationMatrixY( double angle )
{
    double cosAngle = cos( angle );
    double sinAngle = sin( angle );

    cv::Mat rotMtxY = cv::Mat::eye( 3, 3, CV_64FC1 );
    rotMtxY.at<double>( 0, 0 ) = cosAngle;
    rotMtxY.at<double>( 0, 2 ) = sinAngle;
    rotMtxY.at<double>( 2, 0 ) = -sinAngle;
    rotMtxY.at<double>( 2, 2 ) = cosAngle;

    return rotMtxY;
}

//--------------------------------------------------------------------------------------------------
cv::Mat createRotationMatrixZ( double angle )
{
    double cosAngle = cos( angle );
    double sinAngle = sin( angle );

    cv::Mat rotMtxZ = cv::Mat::eye( 3, 3, CV_64FC1 );
    rotMtxZ.at<double>( 0, 0 ) = cosAngle;
    rotMtxZ.at<double>( 0, 1 ) = -sinAngle;
    rotMtxZ.at<double>( 1, 0 ) = sinAngle;
    rotMtxZ.at<double>( 1, 1 ) = cosAngle;

    return rotMtxZ;
}

//--------------------------------------------------------------------------------------------------
cv::Mat createCameraWorldMatrix( const cv::Mat& cameraPos, const cv::Mat& cameraRotXYZDeg )
{
    cv::Mat cameraInAssemblySpaceMtx = cv::Mat::eye( 4, 4, CV_64FC1 );

    double angleX = Utilities::degToRad( cameraRotXYZDeg.at<double>( 0, 0 ) );
    double angleY = Utilities::degToRad( cameraRotXYZDeg.at<double>( 1, 0 ) );
    double angleZ = Utilities::degToRad( cameraRotXYZDeg.at<double>( 2, 0 ) );
    cv::Mat rotMtx = createRotationMatrixZ( angleZ )
        *createRotationMatrixY( angleY )*createRotationMatrixX( angleX );

    cv::Mat rotTarget( cameraInAssemblySpaceMtx, cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat posTarget( cameraInAssemblySpaceMtx, cv::Rect( 3, 0, 1, 3 ) );

    rotMtx.copyTo( rotTarget );
    cameraPos.copyTo( posTarget );

    cv::Mat assemblyInWorldSpaceMtx = cv::Mat::eye( 4, 4, CV_64FC1 );
    cv::Mat assemblyRotTarget( assemblyInWorldSpaceMtx, cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat assemblyRotMtx = createRotationMatrixZ( Utilities::degToRad( 180.0 ) );
    assemblyRotMtx.copyTo( assemblyRotTarget );

    return assemblyInWorldSpaceMtx*cameraInAssemblySpaceMtx;
}

//--------------------------------------------------------------------------------------------------
cv::Mat createChessboardPoseMatrix( const PoseData& poseData )
{
    cv::Mat poseMtx = cv::Mat::eye( 4, 4, CV_64FC1 );

    double angleX = poseData.rotXYZ[ 0 ];
    double angleY = poseData.rotXYZ[ 1 ];
    double angleZ = poseData.rotXYZ[ 2 ];
    cv::Mat rotMtx = createRotationMatrixZ( angleZ )
        *createRotationMatrixY( angleY )*createRotationMatrixX( angleX );

    cv::Mat rotTarget( poseMtx, cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat posTarget( poseMtx, cv::Rect( 3, 0, 1, 3 ) );

    rotMtx.copyTo( rotTarget );
    cv::Mat( poseData.position ).copyTo( posTarget );

    return poseMtx;
}

//--------------------------------------------------------------------------------------------------
ePixelColour getChessboardPixelColour( double u, double v )
{
	ePixelColour pixelColour = ePC_None;

	if ( u >= 0.0 && u <= 1.0 && v >= 0.0 && v <= 1.0 )
	{
		// First work out the colour along the top row
		double borderProportionU = CHESSBOARD_BORDER_WIDTH/CHESSBOARD_TOTAL_WIDTH;

		if ( u <= borderProportionU
			|| u >= 1.0 - borderProportionU )
		{
			// In the border on the u coordinate
			pixelColour = ePC_White;
		}
		else
		{
			int32_t squareIdxU = (int32_t)(((u - borderProportionU)/(1.0 - 2.0*borderProportionU)) * CHESSBOARD_WIDTH);
			ePixelColour topRowColour;
			if ( CHESSBOARD_TOP_LEFT_CORNER_IS_BLACK )
			{
				topRowColour = ( squareIdxU%2 == 0 ? ePC_Black : ePC_White );
			}
			else
			{
				topRowColour = ( squareIdxU%2 == 0 ? ePC_White : ePC_Black );
			}

			// Now find the colour in the column
			double borderProportionV = CHESSBOARD_BORDER_WIDTH/CHESSBOARD_TOTAL_HEIGHT;
			if ( v <= borderProportionV
				|| v >= 1.0 - borderProportionV )
			{
				// In the border on the v coordinate
				pixelColour = ePC_White;
			}
			else
			{
				int32_t squareIdxV = (int32_t)(((v - borderProportionV)/(1.0 - 2.0*borderProportionV)) * CHESSBOARD_HEIGHT);
				if ( ePC_Black == topRowColour )
				{
					pixelColour = ( squareIdxV%2 == 0 ? ePC_Black : ePC_White );
				}
				else
				{
					pixelColour = ( squareIdxV%2 == 0 ? ePC_White : ePC_Black );
				}
			}
		}
	}

	return pixelColour;
}

//--------------------------------------------------------------------------------------------------
ePixelColour getCircleboardPixelColour( double u, double v )
{
    ePixelColour pixelColour = ePC_None;

    if ( u >= 0.0 && u <= CIRCLEBOARD_TOTAL_WIDTH
        && v >= 0.0 && v <= CIRCLEBOARD_TOTAL_HEIGHT )
    {
        // Default to white
        pixelColour = ePC_White;

        // Loop through each of the points on the board, checking to see if the point is
        // close enough to one of them
        double pointV = CIRCLEBOARD_BORDER_WIDTH;

        for ( int32_t y = 0; y < CIRCLEBOARD_HEIGHT; y++ )
        {
            double pointU = CIRCLEBOARD_BORDER_WIDTH;
            for ( int32_t x = 0; x < CIRCLEBOARD_WIDTH; x++ )
            {
                double du = u - pointU;
                double dv = v - pointV;

                if ( du*du + dv*dv <= CIRCLEBOARD_CIRCLE_RADIUS*CIRCLEBOARD_CIRCLE_RADIUS )
                {
                    pixelColour = ePC_Black;
                    goto PointFound;
                }

                pointU += CIRCLEBOARD_SQUARE_SIDE_LENGTH;
            }

            pointV += CIRCLEBOARD_SQUARE_SIDE_LENGTH;
        }
    }

PointFound:

    return pixelColour;
}

//--------------------------------------------------------------------------------------------------
cv::Mat getChessboardPointWorldPos( double u, double v, const cv::Mat& chessboardPoseMtx )
{
	return chessboardPoseMtx.col( 3 )
		+ (u - 0.5)*CHESSBOARD_TOTAL_WIDTH*chessboardPoseMtx.col( 0 )
		- (v - 0.5)*CHESSBOARD_TOTAL_HEIGHT*chessboardPoseMtx.col( 1 );
}

//--------------------------------------------------------------------------------------------------
std::vector<PointData> generateImagePoints( const cv::Mat& cameraWorldMtx, const cv::Mat& cameraCalibMtx,
        int32_t imageWidth, int32_t imageHeight, const cv::Mat& chessboardPoseMtx )
{
    std::vector<PointData> points;
    points.reserve( imageWidth*imageHeight );

    cv::Mat camAxisX( cameraWorldMtx, cv::Rect( 0, 0, 1, 3 ) );
    cv::Mat camAxisY( cameraWorldMtx, cv::Rect( 1, 0, 1, 3 ) );
    cv::Mat camAxisZ( cameraWorldMtx, cv::Rect( 2, 0, 1, 3 ) );
    cv::Mat camPos( cameraWorldMtx, cv::Rect( 3, 0, 1, 3 ) );

    cv::Mat chessboardAxisX( chessboardPoseMtx, cv::Rect( 0, 0, 1, 3 ) );
    cv::Mat chessboardAxisY( chessboardPoseMtx, cv::Rect( 1, 0, 1, 3 ) );
    cv::Mat chessboardAxisZ( chessboardPoseMtx, cv::Rect( 2, 0, 1, 3 ) );
    cv::Mat chessboardPos( chessboardPoseMtx, cv::Rect( 3, 0, 1, 3 ) );

    double chessboardPlaneDist = cv::Mat( chessboardPos.t()*chessboardAxisZ ).at<double>( 0 );

    // Used to calculate plane intersection
    double intersectionConstant = chessboardPlaneDist - cv::Mat(camPos.t()*chessboardAxisZ).at<double>( 0 );

    // Generate the image points using very simple ray tracing
    for ( int32_t y = 0; y < imageHeight; y++ )
    {
        for ( int32_t x = 0; x < imageWidth; x++ )
        {
            double imagePlaneX = (x - cameraCalibMtx.at<double>( 0, 2 )) / cameraCalibMtx.at<double>( 0, 0 );
            double imagePlaneY = (y - cameraCalibMtx.at<double>( 1, 2 )) / cameraCalibMtx.at<double>( 1, 1 );
            cv::Mat rayDir = camAxisZ + imagePlaneX*camAxisX + imagePlaneY*camAxisY;

            double cosOfAngleToChessboard = cv::Mat(rayDir.t()*chessboardAxisZ).at<double>( 0 );
            if ( 0.0 != cosOfAngleToChessboard )
            {
                double distanceToBoard = intersectionConstant/cosOfAngleToChessboard;

                if ( distanceToBoard > 0.0 )
                {
                    cv::Mat intersectionPos = camPos + rayDir*distanceToBoard;
                    cv::Mat fromCentreVec = intersectionPos - chessboardPos;

                    double chessX = cv::Mat(chessboardAxisX.t()*fromCentreVec).at<double>( 0 );
                    double chessY = cv::Mat(chessboardAxisY.t()*fromCentreVec).at<double>( 0 );

                    ePixelColour pixelColour = ePC_None;

                    if ( gbDrawCircles )
                    {
                        double boardU = chessX + 0.5*CHESSBOARD_TOTAL_WIDTH;
                        double boardV = chessY + 0.5*CHESSBOARD_TOTAL_HEIGHT;
                        pixelColour = getCircleboardPixelColour( boardU, boardV );
                    }
                    else
                    {
                        double u = chessX/CHESSBOARD_TOTAL_WIDTH + 0.5;
                        double v = chessY/CHESSBOARD_TOTAL_HEIGHT + 0.5;
                        pixelColour = getChessboardPixelColour( u, v );
                    }

                    if ( ePC_None != pixelColour )
                    {
                        PointData pointData;
                        pointData.mPixelX = x;
                        pointData.mPixelY = y;
                        pointData.mWorldX = intersectionPos.at<double>( 0 );
                        pointData.mWorldY = intersectionPos.at<double>( 1 );
                        pointData.mWorldZ = intersectionPos.at<double>( 2 );
                        pointData.mPixelColour = pixelColour;

                        points.push_back( pointData );
                    }
                }
            }
        }
    }

    return points;
}

//--------------------------------------------------------------------------------------------------
cv::Mat generateRGBImageOfChessboard( const cv::Mat& cameraWorldMtx, const cv::Mat& cameraCalibMtx,
		int32_t imageWidth, int32_t imageHeight, const cv::Mat& chessboardPoseMtx )
{
	cv::Mat image = cv::Mat::zeros( imageHeight, imageWidth, CV_8UC3 );

	std::vector<PointData> points = generateImagePoints(
	    cameraWorldMtx, cameraCalibMtx, imageWidth, imageHeight, chessboardPoseMtx );

	for ( uint32_t pointIdx = 0; pointIdx < points.size(); pointIdx++ )
	{
	    const PointData& point = points[ pointIdx ];
	    cv::Vec3b& pixelData = image.at<cv::Vec3b>( point.mPixelY, point.mPixelX );
	    if ( ePC_Black == point.mPixelColour )
        {
            pixelData[ 0 ] = 0;
            pixelData[ 1 ] = 0;
            pixelData[ 2 ] = 0;
        }
        else if ( ePC_White == point.mPixelColour )
        {
            pixelData[ 0 ] = 255;
            pixelData[ 1 ] = 255;
            pixelData[ 2 ] = 255;
        }
	}

	return image;
}

//--------------------------------------------------------------------------------------------------
PointCloud::Ptr generatePointCloudOfChessboard( const cv::Mat& cameraWorldMtx, const cv::Mat& cameraCalibMtx,
        int32_t imageWidth, int32_t imageHeight, const cv::Mat& chessboardPoseMtx )
{
    double focalLengthPixels = cameraCalibMtx.at<double>( 0, 0 );
    PointCloud::Ptr pCloud( new PointCloud( imageWidth, imageHeight, (float)focalLengthPixels ) );

    std::vector<PointData> points = generateImagePoints(
        cameraWorldMtx, cameraCalibMtx, imageWidth, imageHeight, chessboardPoseMtx );

    for ( uint32_t pointIdx = 0; pointIdx < points.size(); pointIdx++ )
    {
        const PointData& point = points[ pointIdx ];

        uint8_t r, g, b, a;
        if ( ePC_Black == point.mPixelColour )
        {
            r = 0;
            g = 0;
            b = 0;
            a = 255;
        }
        else if ( ePC_White == point.mPixelColour )
        {
            r = 255;
            g = 255;
            b = 255;
            a = 255;
        }
        else
        {
            // Unrecognised colour
            continue;
        }

        pCloud->addPoint( Eigen::Vector3f( (float)point.mWorldX, (float)point.mWorldY, (float)point.mWorldZ ), r, g, b, a );
    }

    return pCloud;
}

//--------------------------------------------------------------------------------------------------
int main( int argc, char** argv )
{
    // Read in a configuration file
    if ( argc < 2 )
    {
        fprintf( stderr, "No configuration file provided\n" );
        showUsage( argv[ 0 ] );
        return -1;
    }

    initialiseTestPositions();

    //std::string configFilename = Utilities::getDataDir() + std::string( "/" ) + std::string( argv[ 1 ] );
    std::string configFilename( argv[ 1 ] );

    cv::FileStorage configFileStorage;
    configFileStorage.open( configFilename, cv::FileStorage::READ );

    if ( !configFileStorage.isOpened() )
    {
        fprintf( stderr, "Unable to open %s\n", configFilename.c_str() );
        return -1;
    }

    cv::Mat kinectDepthCameraPos;
    cv::Mat kinectDepthCameraRotXYZDeg;
    double kinectDepthFocalLengthPixel;
    int32_t kinectDepthImageWidth;
    int32_t kinectDepthImageHeight;

    cv::Mat kinectRGBCameraPos;
    cv::Mat kinectRGBCameraRotXYZDeg;
    double kinectRGBFocalLengthPixel;
    int32_t kinectRGBImageWidth;
    int32_t kinectRGBImageHeight;

    cv::Mat highResCameraPos;
    cv::Mat highResCameraRotXYZDeg;
    double highResFocalLengthPixel;
    int32_t highResImageWidth;
    int32_t highResImageHeight;

    // The camera poses are defined in a coordinate system with the Kinect depth camera at the
    // origin looking down the +ve z-axis, so z values increase into the image. The x and y axes
    // of the depth camera are aligned with its image plane, so +ve x points to its right, and
    // +ve y points down through the base of the camera. To position the camera in world space, it
    // is rotated 180 degrees about the z axis, the chessboard positions are defined in world space.

    configFileStorage[ "kinectDepthFocalLengthPixel" ] >> kinectDepthFocalLengthPixel;
    configFileStorage[ "kinectDepthImageWidth" ] >> kinectDepthImageWidth;
    configFileStorage[ "kinectDepthImageHeight" ] >> kinectDepthImageHeight;

    configFileStorage[ "kinectRGBCameraPos" ] >> kinectRGBCameraPos;
    configFileStorage[ "kinectRGBCameraRotXYZDeg" ] >> kinectRGBCameraRotXYZDeg;
    configFileStorage[ "kinectRGBFocalLengthPixel" ] >> kinectRGBFocalLengthPixel;
    configFileStorage[ "kinectRGBImageWidth" ] >> kinectRGBImageWidth;
    configFileStorage[ "kinectRGBImageHeight" ] >> kinectRGBImageHeight;

    configFileStorage[ "highResCameraPos" ] >> highResCameraPos;
    configFileStorage[ "highResCameraRotXYZDeg" ] >> highResCameraRotXYZDeg;
    configFileStorage[ "highResFocalLengthPixel" ] >> highResFocalLengthPixel;
    configFileStorage[ "highResImageWidth" ] >> highResImageWidth;
    configFileStorage[ "highResImageHeight" ] >> highResImageHeight;

    // Construct camera matrices
    cv::Mat zeroVec = cv::Mat::zeros( 3, 1, CV_64FC1 );
    cv::Mat kinectDepthCalibMtx = createCameraCalibrationMatrix(
        kinectDepthFocalLengthPixel, kinectDepthImageWidth, kinectDepthImageHeight );
    cv::Mat kinectDepthWorldMtx = createCameraWorldMatrix( zeroVec, zeroVec );

    cv::Mat kinectRGBCalibMtx = createCameraCalibrationMatrix(
        kinectRGBFocalLengthPixel, kinectRGBImageWidth, kinectRGBImageHeight );
    cv::Mat kinectRGBWorldMtx = createCameraWorldMatrix(
        kinectRGBCameraPos, kinectRGBCameraRotXYZDeg );

    cv::Mat highResCalibMtx = createCameraCalibrationMatrix(
        highResFocalLengthPixel, highResImageWidth, highResImageHeight );
    cv::Mat highResWorldMtx = createCameraWorldMatrix(
        highResCameraPos, highResCameraRotXYZDeg );

    // First generate calibration images for the high resolution camera
    for ( uint32_t testPosIdx = 0; testPosIdx < gHighResCalibrationPositions.size(); testPosIdx++ )
    {
        cv::Mat chessboardPoseMtx = createChessboardPoseMatrix( gHighResCalibrationPositions[ testPosIdx ] );

        // Generate an image from the high resolution camera
        cv::Mat highResRgbImage = generateRGBImageOfChessboard( highResWorldMtx, highResCalibMtx,
            highResImageWidth, highResImageHeight, chessboardPoseMtx );

        cv::imwrite( createOutputFilename( "chessboard_highres", testPosIdx + 1, ".png" ), highResRgbImage );
    }

    // Now generate images to identify the relative poses of the cameras
    for ( uint32_t testPosIdx = 0; testPosIdx < gRelativeCalibrationPositions.size(); testPosIdx++ )
    {
        cv::Mat chessboardPoseMtx = createChessboardPoseMatrix( gRelativeCalibrationPositions[ testPosIdx ] );

        // Generate a point cloud from the Kinect
        PointCloud::Ptr pCloud = generatePointCloudOfChessboard( kinectDepthWorldMtx, kinectDepthCalibMtx,
            kinectDepthImageWidth, kinectDepthImageHeight, chessboardPoseMtx );

        pCloud->saveToSpcFile( createOutputFilename( "relative_chessboard", testPosIdx + 1, ".spc" ), true );

        // Generate an image from the Kinect RGB camera
        cv::Mat rgbImage = generateRGBImageOfChessboard( kinectRGBWorldMtx, kinectRGBCalibMtx,
            kinectRGBImageWidth, kinectRGBImageHeight, chessboardPoseMtx );

        cv::imwrite( createOutputFilename( "relative_chessboard", testPosIdx + 1, ".png" ), rgbImage );

        // Generate an image from the high resolution camera
        cv::Mat highResRgbImage = generateRGBImageOfChessboard( highResWorldMtx, highResCalibMtx,
            highResImageWidth, highResImageHeight, chessboardPoseMtx );

        cv::imwrite( createOutputFilename( "relative_chessboard_highres", testPosIdx + 1, ".png" ), highResRgbImage );
    }

    // Generate ideal calibration files

    // First the kinect calibration
    cv::FileStorage dataFile( "kinect_calib.yaml", cv::FileStorage::WRITE );

    dataFile << "DepthCameraCalibrationMtx" << kinectDepthCalibMtx;
    dataFile << "ColorCameraCalibrationMtx" << kinectRGBCalibMtx;

    cv::Mat kinectRGBInKinectDepthSpace = (kinectDepthWorldMtx.inv())*kinectRGBWorldMtx;
    dataFile << "DepthToColorCameraRotation" << cv::Mat( kinectRGBInKinectDepthSpace, cv::Rect( 0, 0, 3, 3 ) );
    dataFile << "DepthToColorCameraTranslation" << cv::Mat( kinectRGBInKinectDepthSpace, cv::Rect( 3, 0, 1, 3 ) );

    dataFile.release();

    // Now the high resolution camera calibration
    dataFile = cv::FileStorage( "high_res_calib.yaml", cv::FileStorage::WRITE );
    dataFile << "cameraMatrix" << highResCalibMtx;
    dataFile.release();

    // Finally, the position of the high resolution colour camera in Kinect colour camera space
    dataFile = cv::FileStorage( "colour_stereo_calib.yaml", cv::FileStorage::WRITE );

    cv::Mat highResInKinectRGBSpace = (kinectRGBWorldMtx.inv())*highResWorldMtx;
    dataFile << "R" << cv::Mat( highResInKinectRGBSpace, cv::Rect( 0, 0, 3, 3 ) );
    dataFile << "T" << cv::Mat( highResInKinectRGBSpace, cv::Rect( 3, 0, 1, 3 ) );

    dataFile.release();

    //std::cout << kinectRGBWorldMtx << std::endl;
    //std::cout << highResWorldMtx << std::endl;
    //std::cout << highResInKinectRGBSpace << std::endl;

    return 0;
}
