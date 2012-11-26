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
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
const cv::Vec3d TEST_POSITIONS[] =
{
    cv::Vec3d( 0.0, 0.0, 1.0 )
};

const int32_t NUM_TEST_POSITIONS = sizeof( TEST_POSITIONS )/sizeof( TEST_POSITIONS[ 0 ] );

//--------------------------------------------------------------------------------------------------
void showUsage( const char* programName )
{
    printf( "%s configFilename\n", programName );
    printf( "\tconfigFilename - The name of the configuration file for the camera rig\n" );
}

//--------------------------------------------------------------------------------------------------
cv::Mat createCameraCalibrationMatrix( int32_t focalLengthMM, int32_t imageWidth, int32_t imageHeight )
{
    cv::Mat calibMtx = cv::Mat::eye( 3, 3, CV_64FC1 );

    double focalLength = (double)focalLengthMM/1000.0;
    double principleX = (double)imageWidth/2.0;
    double principleY = (double)imageHeight/2.0;

    calibMtx.at<double>( 0, 0 ) = focalLength;
    calibMtx.at<double>( 1, 1 ) = focalLength;
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
    cv::Mat worldMtx = cv::Mat::zeros( 3, 4, CV_64FC1 );

    double angleX = Utilities::degToRad( cameraRotXYZDeg.at<double>( 0, 0 ) );
    double angleY = Utilities::degToRad( cameraRotXYZDeg.at<double>( 1, 0 ) );
    double angleZ = Utilities::degToRad( cameraRotXYZDeg.at<double>( 2, 0 ) );
    cv::Mat rotMtx = createRotationMatrixZ( angleZ )
        *createRotationMatrixY( angleY )*createRotationMatrixX( angleX );

    cv::Mat col0 = worldMtx.col( 0 );
    cv::Mat col1 = worldMtx.col( 1 );
    cv::Mat col2 = worldMtx.col( 2 );
    cv::Mat col3 = worldMtx.col( 3 );

    rotMtx.col( 0 ).copyTo( col0 );
    rotMtx.col( 1 ).copyTo( col1 );
    rotMtx.col( 2 ).copyTo( col2 );
    cameraPos.copyTo( col3 );

    return worldMtx;
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
    int32_t kinectDepthFocalLengthMM;
    int32_t kinectDepthImageWidth;
    int32_t kinectDepthImageHeight;

    cv::Mat kinectRGBCameraPos;
    cv::Mat kinectRGBCameraRotXYZDeg;
    int32_t kinectRGBFocalLengthMM;
    int32_t kinectRGBImageWidth;
    int32_t kinectRGBImageHeight;

    cv::Mat highResCameraPos;
    cv::Mat highResCameraRotXYZDeg;
    int32_t highResFocalLengthMM;
    int32_t highResImageWidth;
    int32_t highResImageHeight;

    configFileStorage[ "kinectDepthCameraPos" ] >> kinectDepthCameraPos;
    configFileStorage[ "kinectDepthCameraRotXYZDeg" ] >> kinectDepthCameraRotXYZDeg;
    configFileStorage[ "kinectDepthFocalLengthMM" ] >> kinectDepthFocalLengthMM;
    configFileStorage[ "kinectDepthImageWidth" ] >> kinectDepthImageWidth;
    configFileStorage[ "kinectDepthImageHeight" ] >> kinectDepthImageHeight;

    configFileStorage[ "kinectRGBCameraPos" ] >> kinectRGBCameraPos;
    configFileStorage[ "kinectRGBCameraRotXYZDeg" ] >> kinectRGBCameraRotXYZDeg;
    configFileStorage[ "kinectRGBFocalLengthMM" ] >> kinectRGBFocalLengthMM;
    configFileStorage[ "kinectRGBImageWidth" ] >> kinectRGBImageWidth;
    configFileStorage[ "kinectRGBImageHeight" ] >> kinectRGBImageHeight;

    configFileStorage[ "highResCameraPos" ] >> highResCameraPos;
    configFileStorage[ "highResCameraRotXYZDeg" ] >> highResCameraRotXYZDeg;
    configFileStorage[ "highResFocalLengthMM" ] >> highResFocalLengthMM;
    configFileStorage[ "highResImageWidth" ] >> highResImageWidth;
    configFileStorage[ "highResImageHeight" ] >> highResImageHeight;

    // Construct camera matrices
    cv::Mat kinectDepthCalibMtx = createCameraCalibrationMatrix(
        kinectDepthFocalLengthMM, kinectDepthImageWidth, kinectDepthImageHeight );
    cv::Mat kinectDepthWorldMtx = createCameraWorldMatrix(
        kinectDepthCameraPos, kinectDepthCameraRotXYZDeg );

    cv::Mat kinectRGBCalibMtx = createCameraCalibrationMatrix(
        kinectRGBFocalLengthMM, kinectRGBImageWidth, kinectRGBImageHeight );
    cv::Mat kinectRGBWorldMtx = createCameraWorldMatrix(
        kinectRGBCameraPos, kinectRGBCameraRotXYZDeg );

    cv::Mat highResCalibMtx = createCameraCalibrationMatrix(
        highResFocalLengthMM, highResImageWidth, highResImageHeight );
    cv::Mat highResWorldMtx = createCameraWorldMatrix(
        highResCameraPos, highResCameraRotXYZDeg );

    //std::cout << kinectDepthCalibMtx << std::endl;
    //std::cout << kinectDepthWorldMtx << std::endl;
    //std::cout << highResWorldMtx << std::endl;

    // Loop over all test positions and orientations for the chessboard
    for ( int32_t testPosIdx = 0; testPosIdx < NUM_TEST_POSITIONS; testPosIdx++ )
    {
        std::cout << cv::Mat( TEST_POSITIONS[ testPosIdx ] ) << std::endl;

        // Generate a point cloud from the Kinect

        // Generate an image from the Kinect RGB camera

        // Generate an image from the high resolution camera
    }

    return 0;
}
