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
// File: wkg_main_window.cpp
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <vtkRenderWindow.h>
#include <Eigen/Geometry>
#include "wkg_main_window.h"

//--------------------------------------------------------------------------------------------------
// WkgMainWindow
//--------------------------------------------------------------------------------------------------
WkgMainWindow::WkgMainWindow()
	: mpNuiSensor( NULL ),
    mDepthStreamHandle( INVALID_HANDLE_VALUE ),
    mColorStreamHandle( INVALID_HANDLE_VALUE ),
    mNextDepthFrameEventHandle( INVALID_HANDLE_VALUE ),
    mNextColorFrameEventHandle( INVALID_HANDLE_VALUE ),
    mbDepthReceived( false ),
    mbColorReceived( false ),
    mpPixmapItem( NULL ),
	mCurView( eV_RgbCamera )
{
    setupUi( this );

    NuiImageResolutionToSize( IMAGE_RESOLUTION, mImageWidth, mImageHeight );

    if ( !tryToConnectToKinect() )
    {
        this->statusbar->showMessage( tr( "Error: Unable to connect to a Kinect" ) );
    }
    else
    {
        this->statusbar->showMessage( tr( "Ready" ) );
    }

    // Allocate memory for the buffers
    mpDepthBuffer = new uint16_t[ mImageWidth*mImageHeight ];
	mpDepthColorBuffer = new uint8_t[ mImageWidth*mImageHeight*4 ];
    mpColorBuffer = new uint8_t[ mImageWidth*mImageHeight*4 ];

    // Create somewhere to show the current image from the Kinect
    mpScene = new QGraphicsScene( this->graphicsView );
    mpScene->setSceneRect( 0, 0, mImageWidth, mImageHeight );
    this->graphicsView->setScene( mpScene );
    
	// Hook up signals
	connect( this->checkNearMode, SIGNAL( clicked() ), 
		this, SLOT( onCheckNearModeClicked() ) );
	connect( this->cbxView, SIGNAL( currentIndexChanged( const QString& ) ), 
		this, SLOT( onCbxViewCurrentIndexChanged( const QString& ) ) );
	connect( this->btnGrabFrame, SIGNAL( clicked() ), this, SLOT( onBtnGrabFrameClicked() ) );

	// Default the view to the RGB Camera
	this->cbxView->setCurrentIndex( 0 );

	tryToSetNearMode( false );

    // Start a time which will call our update event
    startTimer( 1000 / 30 );    // Try to run at 30 fps
}

//--------------------------------------------------------------------------------------------------
WkgMainWindow::~WkgMainWindow()
{
	if ( NULL != mpNuiSensor )
    {
        mpNuiSensor->NuiShutdown();
        mpNuiSensor->Release();
        mpNuiSensor = NULL;
    }

    if ( INVALID_HANDLE_VALUE != mNextDepthFrameEventHandle )
    {
        CloseHandle( mNextDepthFrameEventHandle );
        mNextDepthFrameEventHandle = INVALID_HANDLE_VALUE;
    }

    if ( INVALID_HANDLE_VALUE != mNextColorFrameEventHandle )
    {
        CloseHandle( mNextColorFrameEventHandle );
        mNextColorFrameEventHandle = INVALID_HANDLE_VALUE;
    }

    if ( NULL != mpScene )
    {
        delete mpScene;
        mpScene = NULL;
    }
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onCheckNearModeClicked( bool bChecked )
{
	tryToSetNearMode( this->checkNearMode->isChecked() );
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onCbxViewCurrentIndexChanged( const QString& text )
{
	if ( "RGB Camera" == text )
	{
		mCurView = eV_RgbCamera;
	}
	else if ( "Depth Camera" == text ) 
	{
		mCurView = eV_DepthCamera;
	}
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onBtnGrabFrameClicked()
{
}

//--------------------------------------------------------------------------------------------------
bool WkgMainWindow::tryToConnectToKinect()
{
    INuiSensor* pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount( &iSensorCount );
    if ( FAILED( hr ) )
    {
        return false;
    }

    // Look at each Kinect sensor
    for ( int i = 0; i < iSensorCount; ++i )
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex( i, &pNuiSensor );
        if ( FAILED( hr ) )
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if ( S_OK == hr )
        {
            mpNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if ( NULL != mpNuiSensor )
    {
        // Initialize the Kinect and specify that we'll be using depth
        hr = mpNuiSensor->NuiInitialize( //NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX 
			NUI_INITIALIZE_FLAG_USES_DEPTH 
			| NUI_INITIALIZE_FLAG_USES_COLOR ); 
        if ( SUCCEEDED( hr ) )
        {
            // Create an event that will be signaled when depth data is available
            mNextDepthFrameEventHandle = CreateEvent( NULL, TRUE, FALSE, NULL );

            // Open a depth image stream to receive depth frames
            hr = mpNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH,
                IMAGE_RESOLUTION,
                0,
                2,
                mNextDepthFrameEventHandle,
                &mDepthStreamHandle );

            if ( FAILED(hr) ) 
            { 
                return false; 
            }

            // Create an event that will be signaled when color data is available
            mNextColorFrameEventHandle = CreateEvent( NULL, TRUE, FALSE, NULL );

            // Open a color image stream to receive color frames
            hr = mpNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_COLOR,
                IMAGE_RESOLUTION,
                0,
                2,
                mNextColorFrameEventHandle,
                &mColorStreamHandle );

            if ( FAILED(hr) ) 
            { 
                return false; 
            }
        }
    }

    if ( NULL == mpNuiSensor || FAILED( hr ) )
    {
        return false;
    }

    // The Kinect is now set up
    return true;
}

//--------------------------------------------------------------------------------------------------
HRESULT WkgMainWindow::tryToSetNearMode( bool bOn )
{
	HRESULT hr = E_FAIL;
	this->checkNearMode->setChecked( false );

    if ( mpNuiSensor )
    {
        hr = mpNuiSensor->NuiImageStreamSetImageFrameFlags(
			mDepthStreamHandle, bOn ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0 );

        if ( SUCCEEDED(hr) )
        {
            this->checkNearMode->setChecked( bOn );
        }
    }

    return hr;
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::update()
{
    bool needToMapColorToDepth = false;

    if ( WAIT_OBJECT_0 == WaitForSingleObject( mNextDepthFrameEventHandle, 0 ) )
    {
        // If we have received any valid new depth data we may need to draw
        if ( SUCCEEDED( processDepth() ) )
        {
            needToMapColorToDepth = true;
        }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject( mNextColorFrameEventHandle, 0 ) )
    {
        // If we have received any valid new color data we may need to draw
        if ( SUCCEEDED( processColor() ) )
        {
            needToMapColorToDepth = true;
        }
    }

    // If we have not yet received any data for either color or depth since we started up, we shouldn't draw
    if ( !mbDepthReceived || !mbColorReceived )
    {
        needToMapColorToDepth = false;
    }

    if ( needToMapColorToDepth )
    {
        mapColorToDepth();
		drawImage();
    }
}

//--------------------------------------------------------------------------------------------------
HRESULT WkgMainWindow::processDepth()
{
    NUI_IMAGE_FRAME imageFrame;

    HRESULT hr = mpNuiSensor->NuiImageStreamGetNextFrame( mDepthStreamHandle, 0, &imageFrame );
    if ( FAILED(hr) ) { return hr; }
   
    NUI_LOCKED_RECT lockedRect;
    hr = imageFrame.pFrameTexture->LockRect( 0, &lockedRect, NULL, 0 );
    if ( FAILED(hr) ) { return hr; }

    memcpy( mpDepthBuffer, lockedRect.pBits, lockedRect.size );
    mbDepthReceived = true;

    hr = imageFrame.pFrameTexture->UnlockRect( 0 );
    if ( FAILED(hr) ) { return hr; };

    hr = mpNuiSensor->NuiImageStreamReleaseFrame( mDepthStreamHandle, &imageFrame );

	// Convert each point into skeleton space. Looking at the multiplier used to do the transform
	int numPoints = 0;
	float xScaleMean = 0.0f;
	float xScaleM2 = 0.0f;
	float yScaleMean = 0.0f;
	float yScaleM2 = 0.0f;
	float zScaleMean = 0.0f;
	float zScaleM2 = 0.0f;

	for ( int y = 0; y < mImageHeight; y++ )
	{
		for ( int x = 0; x < mImageWidth; x++ )
		{
			uint16_t depthValue = mpDepthBuffer[ y*mImageWidth + x ];

			float screenCentreX = ((float)mImageWidth - 0.0)/2.0;
			float screenCentreY = ((float)mImageHeight - 0.0)/2.0;

			if ( 0 != depthValue
				&& 0 != ((float)x - screenCentreX) && 0 != ((float)y - screenCentreY) )
			{
				Vector4 v = NuiTransformDepthImageToSkeleton( x, y, depthValue, IMAGE_RESOLUTION );

				numPoints++;
				float xScale = (v.x/v.z) / ((float)x - screenCentreX);
				float yScale = (v.y/v.z) / ((float)y - screenCentreY);
				float zScale = v.z / (float)depthValue;

				float xScaleDelta = xScale - xScaleMean;
				xScaleMean += xScaleDelta/numPoints;
				xScaleM2 += xScaleDelta*(xScale - xScaleMean);

				float yScaleDelta = yScale - yScaleMean;
				yScaleMean += yScaleDelta/numPoints;
				yScaleM2 += yScaleDelta*(yScale - yScaleMean);

				float zScaleDelta = zScale - zScaleMean;
				zScaleMean += zScaleDelta/numPoints;
				zScaleM2 += zScaleDelta*(zScale - zScaleMean);
			}
		}
	}

	if ( numPoints > 0 )
	{
		printf( "xScale mean %2.8f var %2.8f   yScale mean %2.8f var %2.8f   zScale mean %2.4f var %2.4f\n", 
			xScaleMean, xScaleM2/numPoints, yScaleMean, yScaleM2/numPoints, zScaleMean, zScaleM2/numPoints );
	}
	else
	{
		printf( "No depth points\n" );
	}

    return hr;
}

//--------------------------------------------------------------------------------------------------
HRESULT WkgMainWindow::processColor()
{
    NUI_IMAGE_FRAME imageFrame;

    HRESULT hr = mpNuiSensor->NuiImageStreamGetNextFrame( mColorStreamHandle, 0, &imageFrame );
    if ( FAILED(hr) ) { return hr; }
  
    NUI_LOCKED_RECT lockedRect;
    hr = imageFrame.pFrameTexture->LockRect( 0, &lockedRect, NULL, 0 );
    if ( FAILED(hr) ) { return hr; }

    memcpy( mpColorBuffer, lockedRect.pBits, lockedRect.size );
    mbColorReceived = true;

    hr = imageFrame.pFrameTexture->UnlockRect( 0 );
    if ( FAILED(hr) ) { return hr; };

    hr = mpNuiSensor->NuiImageStreamReleaseFrame( mColorStreamHandle, &imageFrame );

    return hr;
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::mapColorToDepth()
{
	std::vector<LONG> colorCoordinates;
	colorCoordinates.resize( mImageWidth*mImageHeight*2, 0 );
	
    // Get the coordinates of every depth pixel in colour space
    mpNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
        IMAGE_RESOLUTION,
        IMAGE_RESOLUTION,
        mImageWidth*mImageHeight,
		mpDepthBuffer,
        mImageWidth*mImageHeight*2,
        &colorCoordinates[ 0 ] );

    // Get the color for each depth pixel
	for ( DWORD i = 0; i < mImageWidth*mImageHeight; i++ )
	{
		LONG colorX = colorCoordinates[ i*2 ];
		LONG colorY = colorCoordinates[ i*2 + 1 ];

		if ( 0 != mpDepthBuffer[ i ] 
			&& colorX >= 0 && colorX < (LONG)mImageWidth
			&& colorY >= 0 && colorY < (LONG)mImageHeight )
		{
			DWORD colorPixelIdx = colorY*mImageWidth + colorX;
			mpDepthColorBuffer[ 4*i ] = mpColorBuffer[ 4*colorPixelIdx ];
			mpDepthColorBuffer[ 4*i + 1 ] = mpColorBuffer[ 4*colorPixelIdx + 1 ];
			mpDepthColorBuffer[ 4*i + 2 ] = mpColorBuffer[ 4*colorPixelIdx + 2 ];
			mpDepthColorBuffer[ 4*i + 3 ] = mpColorBuffer[ 4*colorPixelIdx + 3 ];
		}
		else
		{
			// No color so set the pixel to black
			mpDepthColorBuffer[ 4*i ] = 0;
			mpDepthColorBuffer[ 4*i + 1 ] = 0;
			mpDepthColorBuffer[ 4*i + 2 ] = 0;
			mpDepthColorBuffer[ 4*i + 3 ] = 0;
		}
	}
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::drawImage()
{
	uint8_t* pImageBuffer = mpColorBuffer;

	if ( eV_DepthCamera == mCurView )
	{
		pImageBuffer = mpDepthColorBuffer;
	}

	// Draw the color image
	QPixmap pixmap( mImageWidth, mImageHeight );
	QImage image( pImageBuffer, mImageWidth, mImageHeight, QImage::Format_RGB32 );

	// Mirror the image horizontally for diaply
	pixmap.convertFromImage( image.mirrored( true, false ) );

	if ( NULL == mpPixmapItem )
	{
		mpPixmapItem = mpScene->addPixmap( pixmap );
	}
	else
	{
		mpPixmapItem->setPixmap( pixmap );
	}
}