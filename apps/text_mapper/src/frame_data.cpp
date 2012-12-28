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
// File: frame_data.cpp
// Desc: A simple struct for holding the properties and data of a frame
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <QtGui/qmessagebox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "frame_data.h"

//--------------------------------------------------------------------------------------------------
// FrameData
//--------------------------------------------------------------------------------------------------
bool FrameData::tryToLoadImages( bool bShowErrorMsgBox )
{
    // Try to load in the images
    mHighResImage = cv::imread( mHighResImageFilename );
    if ( NULL == mHighResImage.data )
    {
        if ( bShowErrorMsgBox )
        {
            QMessageBox::critical( NULL, "Error", "Unable to load high resolution image" );
        }
        return false;
    }
    cv::cvtColor( mHighResImage, mHighResImage, CV_BGR2RGB );

    mKinectColorImage = cv::imread( mKinectColorImageFilename );
    if ( NULL == mKinectColorImage.data )
    {
        if ( bShowErrorMsgBox )
        {
            QMessageBox::critical( NULL, "Error", "Unable to load Kinect color image" );
        }
        return false;
    }
    cv::cvtColor( mKinectColorImage, mKinectColorImage, CV_BGR2RGB );

    // Try to load in the point cloud
    mpKinectDepthPointCloud = PointCloud::loadPointCloudFromSpcFile( mKinectDepthPointCloudFilename );
    if ( NULL == mpKinectDepthPointCloud )
    {
        if ( bShowErrorMsgBox )
        {
            QMessageBox::critical( NULL, "Error", "Unable to load Kinect point cloud" );
        }
        return false;
    }

    return true;
}

//--------------------------------------------------------------------------------------------------
void FrameData::unloadImages()
{
    mHighResImage = cv::Mat();
    mKinectColorImage = cv::Mat();
    mpKinectDepthPointCloud = PointCloud::Ptr();
}

