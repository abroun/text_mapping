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
    connect( this->buttonBox, SIGNAL( accepted() ), this, SLOT( onAccept() ) );
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
        printf( "Accepted!\n" );
		*pFrameDataOut = dialog.getFrameData();
        bFrameCreated = true;
	}
    else
    {
        printf( "Rejected...\n" );
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

	QString filename = QFileDialog::getSaveFileName( this,
		tr( "High Res Image" ), baseDir.c_str(), tr("Images (*.jpeg *.jpg *.png *.bmp)") );
	
	if ( !filename.isEmpty() )
	{
		editHighResImageFilename->setText( filename );
		mFrameData.mHighResImageFilename = filename.toStdString();
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onBtnChooseKinectColorImageClicked()
{
	std::string baseDir = Utilities::getDataDir() + "/images/kinect";

	QString filename = QFileDialog::getSaveFileName( this,
		tr( "Kinect Color Image" ), baseDir.c_str(), tr("Images (*.jpeg *.jpg *.png *.bmp)") );
	
	if ( !filename.isEmpty() )
	{
		editKinectColorImageFilename->setText( filename );
		mFrameData.mKinectColorImageFilename = filename.toStdString();
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onBtnChooseKinectDepthPointCloudClicked()
{
	std::string baseDir = Utilities::getDataDir() + "/point_clouds";

	QString filename = QFileDialog::getSaveFileName( this,
		tr( "Kinect Depth Point Cloud" ), baseDir.c_str(), tr("Simple Point Cloud (*.spc)") );
	
	if ( !filename.isEmpty() )
	{
		editKinectDepthPointCloudFilename->setText( filename );
		mFrameData.mKinectDepthPointCloudFilename = filename.toStdString();
	}
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::onAccept()
{
    this->setResult( QDialog::Accepted );
}

//--------------------------------------------------------------------------------------------------
void FrameDialog::setFrameData( const FrameData& frameData )
{
	mFrameData = frameData;
}

