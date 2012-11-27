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
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
struct PoseData
{
    cv::Vec3d position;
    cv::Vec3d rotXYZ;       // Used to specify orientation assuming that normal starts as +ve z-axis
};

//--------------------------------------------------------------------------------------------------
const PoseData TEST_POSITIONS[] =
{
    { cv::Vec3d( 0.0, 0.0, 1.0 ), cv::Vec3d( 0.0, Utilities::degToRad( 180.0 ), 0.0 ) },
    { cv::Vec3d( 0.1, 0.0, 1.0 ), cv::Vec3d( 0.0, Utilities::degToRad( 140.0 ), 0.0 ) }
};

const int32_t NUM_TEST_POSITIONS = sizeof( TEST_POSITIONS )/sizeof( TEST_POSITIONS[ 0 ] );

enum ePixelColour
{
	ePC_None = -1,
	ePC_Black = 0,
	ePC_White
};

// TODO: Move these to the config file
const double CHESSBOARD_SQUARE_SIDE_LENGTH = 0.02;
const double CHESSBOARD_BORDER_WIDTH = 0.04;
const int32_t CHESSBOARD_WIDTH = 9;
const int32_t CHESSBOARD_HEIGHT = 7;
const bool CHESSBOARD_TOP_LEFT_CORNER_IS_BLACK = true;

//--------------------------------------------------------------------------------------------------
void showUsage( const char* programName )
{
    printf( "%s configFilename\n", programName );
    printf( "\tconfigFilename - The name of the configuration file for the camera rig\n" );
}

//--------------------------------------------------------------------------------------------------
cv::Mat createCameraCalibrationMatrix( double focalLengthPixel, int32_t imageWidth, int32_t imageHeight )
{
    cv::Mat calibMtx = cv::Mat::eye( 3, 3, CV_64FC1 );

    double principleX = (double)imageWidth/2.0;
    double principleY = (double)imageHeight/2.0;

    calibMtx.at<double>( 0, 0 ) = -focalLengthPixel;
    calibMtx.at<double>( 1, 1 ) = -focalLengthPixel;
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
    cv::Mat worldMtx = cv::Mat::eye( 4, 4, CV_64FC1 );

    double angleX = Utilities::degToRad( cameraRotXYZDeg.at<double>( 0, 0 ) );
    double angleY = Utilities::degToRad( cameraRotXYZDeg.at<double>( 1, 0 ) );
    double angleZ = Utilities::degToRad( cameraRotXYZDeg.at<double>( 2, 0 ) );
    cv::Mat rotMtx = createRotationMatrixZ( angleZ )
        *createRotationMatrixY( angleY )*createRotationMatrixX( angleX );

    cv::Mat rotTarget( worldMtx, cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat posTarget( worldMtx, cv::Rect( 3, 0, 1, 3 ) );

    rotMtx.copyTo( rotTarget );
    cameraPos.copyTo( posTarget );

    return worldMtx;
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
		double totalWidth = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_WIDTH*CHESSBOARD_SQUARE_SIDE_LENGTH;
		double borderProportionU = CHESSBOARD_BORDER_WIDTH/totalWidth;

		if ( u <= borderProportionU
			|| u >= 1.0 - borderProportionU )
		{
			// In the border on the u coordinate
			pixelColour = ePC_White;
		}
		else
		{
			int32_t squareIdxU = ((u - borderProportionU)/(1.0 - 2.0*borderProportionU)) * CHESSBOARD_WIDTH;
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
			double totalHeight = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_HEIGHT*CHESSBOARD_SQUARE_SIDE_LENGTH;
			double borderProportionV = CHESSBOARD_BORDER_WIDTH/totalHeight;
			if ( v <= borderProportionV
				|| v >= 1.0 - borderProportionV )
			{
				// In the border on the v coordinate
				pixelColour = ePC_White;
			}
			else
			{
				int32_t squareIdxV = ((v - borderProportionV)/(1.0 - 2.0*borderProportionV)) * CHESSBOARD_HEIGHT;
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
cv::Mat getChessboardPointWorldPos( double u, double v, const cv::Mat& chessboardPoseMtx )
{
    double totalWidth = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_WIDTH*CHESSBOARD_SQUARE_SIDE_LENGTH;
    double totalHeight = 2.0*CHESSBOARD_BORDER_WIDTH + CHESSBOARD_HEIGHT*CHESSBOARD_SQUARE_SIDE_LENGTH;

	return chessboardPoseMtx.col( 3 )
		+ (u - 0.5)*totalWidth*chessboardPoseMtx.col( 0 )
		- (v - 0.5)*totalHeight*chessboardPoseMtx.col( 1 );
}

//--------------------------------------------------------------------------------------------------
cv::Mat projectWorldPosToImage( const cv::Mat& worldPos, const cv::Mat& worldInCameraSpaceMtx,
		const cv::Mat& cameraCalibMtx )
{
	cv::Mat imagePlanePos = worldInCameraSpaceMtx.rowRange( 0, 3 )*worldPos;
	imagePlanePos /= imagePlanePos.at<double>( 2 );

	// TODO: Implement radial distortion here

	cv::Mat screenPos = cameraCalibMtx.rowRange( 0, 2 )*imagePlanePos;
	return screenPos;
}

//--------------------------------------------------------------------------------------------------
cv::Mat generateRGBImageOfChessboard( const cv::Mat& cameraWorldMtx, const cv::Mat& cameraCalibMtx,
		int32_t imageWidth, int32_t imageHeight, const cv::Mat& chessboardPoseMtx )
{
	cv::Mat image = cv::Mat::zeros( imageHeight, imageWidth, CV_8UC3 );

	// We project the corners of the chessboard into the image to work out the rough step size. This
	// is rather brain-dead, but allows me to easily incorporate the simulation of radial distortion
	// at some point.
	// TODO: Put in the effort to translate the rendering to OpenGL using a shader for distortion.
	// (The challenge here is keeping the code and build process cross platform...)
	cv::Mat worldInCameraSpaceMtx = cameraWorldMtx.inv();


	cv::Mat topLeftScreenPos = projectWorldPosToImage(
	    getChessboardPointWorldPos( 0.0, 0.0, chessboardPoseMtx ),
	    worldInCameraSpaceMtx, cameraCalibMtx );

	cv::Mat topRightScreenPos = projectWorldPosToImage(
	        getChessboardPointWorldPos( 1.0, 0.0, chessboardPoseMtx ),
	        worldInCameraSpaceMtx, cameraCalibMtx );

	cv::Mat bottomLeftScreenPos = projectWorldPosToImage(
            getChessboardPointWorldPos( 0.0, 1.0, chessboardPoseMtx ),
            worldInCameraSpaceMtx, cameraCalibMtx );

	cv::Mat bottomRightScreenPos = projectWorldPosToImage(
            getChessboardPointWorldPos( 1.0, 1.0, chessboardPoseMtx ),
            worldInCameraSpaceMtx, cameraCalibMtx );

	// Work out the max distance in pixels vertically and horizontally
	double maxHorizontalDistance = std::max(
	    cv::norm( topRightScreenPos - topLeftScreenPos ),
	    cv::norm( bottomRightScreenPos - bottomLeftScreenPos ) );
	double maxVerticalDistance = std::max(
        cv::norm( bottomLeftScreenPos - topLeftScreenPos ),
        cv::norm( bottomRightScreenPos - topRightScreenPos ) );

	double stepU = (1.0/maxHorizontalDistance)/1.2;
	double stepV = (1.0/maxVerticalDistance)/1.2;

	for ( double u = 0.0; u <= 1.0; u += stepU )
	{
	    for ( double v = 0.0; v <= 1.0; v += stepV )
	    {
	        ePixelColour pixelColour = getChessboardPixelColour( u, v );
	        if ( ePC_None != pixelColour )
	        {
	            cv::Mat screenPos = projectWorldPosToImage(
	                getChessboardPointWorldPos( u, v, chessboardPoseMtx ),
	                worldInCameraSpaceMtx, cameraCalibMtx );

	            int32_t x = (int32_t)screenPos.at<double>( 0 );
	            int32_t y = (int32_t)screenPos.at<double>( 1 );
	            if ( x >= 0 && x < imageWidth
	                && y >= 0 && y < imageHeight )
	            {
	                cv::Vec3b& pixelData = image.at<cv::Vec3b>( y, x );
                    if ( ePC_White == pixelColour )
                    {
                        pixelData[ 0 ] = 255;
                        pixelData[ 1 ] = 255;
                        pixelData[ 2 ] = 255;
                    }
                    else
                    {
                        pixelData[ 0 ] = 0;
                        pixelData[ 1 ] = 0;
                        pixelData[ 2 ] = 0;
                    }
	            }
	        }
	    }
	}


	return image;
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

    configFileStorage[ "kinectDepthCameraPos" ] >> kinectDepthCameraPos;
    configFileStorage[ "kinectDepthCameraRotXYZDeg" ] >> kinectDepthCameraRotXYZDeg;
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
    cv::Mat kinectDepthCalibMtx = createCameraCalibrationMatrix(
        kinectDepthFocalLengthPixel, kinectDepthImageWidth, kinectDepthImageHeight );
    cv::Mat kinectDepthWorldMtx = createCameraWorldMatrix(
        kinectDepthCameraPos, kinectDepthCameraRotXYZDeg );

    cv::Mat kinectRGBCalibMtx = createCameraCalibrationMatrix(
        kinectRGBFocalLengthPixel, kinectRGBImageWidth, kinectRGBImageHeight );
    cv::Mat kinectRGBWorldMtx = createCameraWorldMatrix(
        kinectRGBCameraPos, kinectRGBCameraRotXYZDeg );

    cv::Mat highResCalibMtx = createCameraCalibrationMatrix(
        highResFocalLengthPixel, highResImageWidth, highResImageHeight );
    cv::Mat highResWorldMtx = createCameraWorldMatrix(
        highResCameraPos, highResCameraRotXYZDeg );

    //std::cout << kinectDepthCalibMtx << std::endl;
    //std::cout << kinectDepthWorldMtx << std::endl;
    //std::cout << highResWorldMtx << std::endl;

    // Loop over all test positions and orientations for the chessboard
    for ( int32_t testPosIdx = 0; testPosIdx < NUM_TEST_POSITIONS; testPosIdx++ )
    {
        cv::Mat chessboardPoseMtx = createChessboardPoseMatrix( TEST_POSITIONS[ testPosIdx ] );

        cv::Mat rgbImage = generateRGBImageOfChessboard( kinectDepthWorldMtx, kinectDepthCalibMtx,
            kinectDepthImageWidth, kinectDepthImageHeight, chessboardPoseMtx );

        cv::imwrite( "chessboard.png", rgbImage );

        cv::Mat highResRgbImage = generateRGBImageOfChessboard( highResWorldMtx, highResCalibMtx,
            highResImageWidth, highResImageHeight, chessboardPoseMtx );

        cv::imwrite( "highres_chessboard.png", highResRgbImage );

        // Generate a point cloud from the Kinect

        // Generate an image from the Kinect RGB camera

        // Generate an image from the high resolution camera
    }

    return 0;
}
