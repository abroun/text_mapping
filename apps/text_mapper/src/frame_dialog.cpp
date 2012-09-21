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
// File: frame_dialog.cpp
// Desc: A dialog which allows the user to set the properties of a frame
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <QtGui/qfiledialog.h>
#include <QtGui/qmessagebox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "text_mapping/utilities.h"
#include "frame_dialog.h"

//--------------------------------------------------------------------------------------------------
// FrameDialog
//--------------------------------------------------------------------------------------------------
FrameDialog::FrameDialog()
{
    setupUi( this );

	// Hook up signals
	connect( this->btnChooseHighResImage, SIGNAL( clicked() ), this, SLOT( onBtnChooseHighResImageClicked() ) );
	connect( this->btnChooseKinectColorImage, SIGNAL( clicked() ), this, SLOT( onBtnChooseKinectColorImageClicked() ) );
	connect( this->btnChooseKinectDepthPointCloud, SIGNAL( clicked() ), this, SLOT( onBtnChooseKinectDepthPointCloudClicked() ) );
}

//--------------------------------------------------------------------------------------------------
FrameDialog::~FrameDialog()
{
}

//--------------------------------------------------------------------------------------------------
bool FrameDialog::createNewFrame( FrameData* pFrameDataOut )
{
	bool bFrameCreated = false;
	FrameDialog dialog;

    if ( dialog.exec() == QDialog::Accepted )
	{
		*pFrameDataOut = dialog.getFrameData();
        bFrameCreated = true;
	}

    return bFrameCreated;
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::editFrame( FrameData* pFrameDataInOut )
{
	FrameDialog dialog;
	dialog.setFrameData( *pFrameDataInOut );
	
	if ( dialog.exec() == QDialog::Accepted )
	{
		*pFrameDataInOut = dialog.getFrameData();
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onBtnChooseHighResImageClicked()
{
	std::string baseDir = Utilities::getDataDir() + "/images/high_res";

	QString filename = QFileDialog::getOpenFileName( this,
		tr( "High Res Image" ), baseDir.c_str(), tr("Images (*.jpeg *.jpg *.png *.bmp)") );
	
	if ( !filename.isEmpty() )
	{
		this->editHighResImageFilename->setText( filename );
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onBtnChooseKinectColorImageClicked()
{
	std::string baseDir = Utilities::getDataDir() + "/images/kinect";

	QString filename = QFileDialog::getOpenFileName( this,
		tr( "Kinect Color Image" ), baseDir.c_str(), tr("Images (*.jpeg *.jpg *.png *.bmp)") );
	
	if ( !filename.isEmpty() )
	{
		this->editKinectColorImageFilename->setText( filename );
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onBtnChooseKinectDepthPointCloudClicked()
{
	std::string baseDir = Utilities::getDataDir() + "/point_clouds";

    QString filename = QFileDialog::getOpenFileName( this,
		tr( "Kinect Depth Point Cloud" ), baseDir.c_str(), tr("Simple Point Cloud (*.spc)") );
	
	if ( !filename.isEmpty() )
	{
		this->editKinectDepthPointCloudFilename->setText( filename );
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::accept()
{
    mFrameData.mHighResImageFilename = this->editHighResImageFilename->text().toStdString();
    mFrameData.mKinectColorImageFilename = this->editKinectColorImageFilename->text().toStdString();
    mFrameData.mKinectDepthPointCloudFilename = this->editKinectDepthPointCloudFilename->text().toStdString();

    // Try to load in the images
    mFrameData.mHighResImage = cv::imread( mFrameData.mHighResImageFilename );
    if ( NULL == mFrameData.mHighResImage.data )
    {
        QMessageBox::critical( this, "Error", "Unable to load high resolution image" );
        return;
    }
    cv::cvtColor( mFrameData.mHighResImage, mFrameData.mHighResImage, CV_BGR2RGB );

    mFrameData.mKinectColorImage = cv::imread( mFrameData.mKinectColorImageFilename );
    if ( NULL == mFrameData.mKinectColorImage.data )
    {
        QMessageBox::critical( this, "Error", "Unable to load Kinect color image" );
        return;
    }
    cv::cvtColor( mFrameData.mKinectColorImage, mFrameData.mKinectColorImage, CV_BGR2RGB );

    // Try to load in the point cloud
    mFrameData.mpKinectDepthPointCloud = PointCloud::loadTextMapFromSpcFile( mFrameData.mKinectDepthPointCloudFilename );
    if ( NULL == mFrameData.mpKinectDepthPointCloud )
    {
    	QMessageBox::critical( this, "Error", "Unable to load Kinect point cloud" );
		return;
    }

    QDialog::accept();
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::setFrameData( const FrameData& frameData )
{
	mFrameData = frameData;
    mFrameData.mHighResImage = cv::Mat();
    mFrameData.mKinectColorImage = cv::Mat();
    mFrameData.mpKinectDepthPointCloud = PointCloud::Ptr();

    this->editHighResImageFilename->setText( mFrameData.mHighResImageFilename.c_str() );
    this->editKinectColorImageFilename->setText( mFrameData.mKinectColorImageFilename.c_str() );
    this->editKinectDepthPointCloudFilename->setText( mFrameData.mKinectDepthPointCloudFilename.c_str() );
}

