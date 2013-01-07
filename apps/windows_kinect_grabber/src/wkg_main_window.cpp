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
#include <iostream>
#include <vtkRenderWindow.h>
#include <Eigen/Geometry>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <QtGui/qfiledialog.h>
#include "wkg_main_window.h"
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
static boost::random::mt19937 gRandomNumberGenerator;

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
	mCurView( eV_RgbCamera ),
    mbHaveValidCameraMatrices( false )
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
    connect( this->btnGrabCalibrationImage, SIGNAL( clicked() ), this, SLOT( onBtnGrabCalibrationImageClicked() ) );
    connect( this->btnClearCalibrationImages, SIGNAL( clicked() ), this, SLOT( onBtnClearCalibrationImagesClicked() ) );
    connect( this->btnCalculateCameraMatrices, SIGNAL( clicked() ), this, SLOT( onBtnCalculateCameraMatricesClicked() ) );
    connect( this->btnSaveCameraMatrices, SIGNAL( clicked() ), this, SLOT( onBtnSaveCameraMatricesClicked() ) );    
    
	// Default the view to the RGB Camera
	this->cbxView->setCurrentIndex( 0 );

	tryToSetNearMode( false );
    updateNumCalibrationImagesDisplay();

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
	if ( mbColorReceived && mbDepthReceived )
    {
        // Take a copy of the current color and depth images
		cv::Mat colorImage( mImageHeight, mImageWidth, CV_8UC4 );
		memcpy( colorImage.data, mpColorBuffer, mImageWidth*mImageHeight*4 );

        cv::Mat depthData( mImageHeight, mImageWidth, CV_16U );
        memcpy( depthData.data, mpDepthBuffer, mImageWidth*mImageHeight*sizeof( uint16_t ) );

        cv::Mat depthColorImage( mImageHeight, mImageWidth, CV_8UC4 );
        memcpy( depthColorImage.data, mpDepthColorBuffer, mImageWidth*mImageHeight*4 );

        // Flip the images about the vertical
        cv::flip( colorImage, colorImage, 1 );
        cv::flip( depthData, depthData, 1 );
        cv::flip( depthColorImage, depthColorImage, 1 );

		// First save the color image
		std::string baseDir = Utilities::getDataDir() + "/images/kinect";
		QString filename = QFileDialog::getSaveFileName( this,
            tr( "Save Color Image" ), baseDir.c_str(), tr( "Image Files (*.jpg *.bmp)" ) );

		if ( !filename.isEmpty() )
        {
			boost::filesystem::path filePath( filename.toStdString() );
			if ( filePath.extension() != ".jpg" && filePath.extension() != ".bmp" )
			{
				filePath.replace_extension( ".jpg" );
			}

			cv::imwrite( filePath.string(), colorImage );
			
			// Next save the depth data as a point cloud
			baseDir = Utilities::getDataDir() + "/point_clouds";
			filename = QFileDialog::getSaveFileName( this,
				tr( "Save Point Cloud" ), baseDir.c_str(), tr( "Simple Point Cloud (*.spc)" ) );

			if ( !filename.isEmpty() )
			{
				filePath = boost::filesystem::path( filename.toStdString() );
				if ( filePath.extension() != ".spc" )
				{
					filePath.replace_extension( ".spc" );
				}

                saveDepthDataToSpcFile( filePath.string(), (uint16_t*)depthData.data, depthColorImage.data, true );
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onBtnGrabCalibrationImageClicked()
{
    if ( mbColorReceived && mbDepthReceived )
    {
        // Copy the current color and depth images
        FrameData frameData;
        frameData.mColorBuffer = cv::Mat( mImageHeight, mImageWidth, CV_8UC4 );
        frameData.mDepthBuffer = cv::Mat( mImageHeight, mImageWidth, CV_16U );

        memcpy( frameData.mColorBuffer.data, mpColorBuffer, mImageWidth*mImageHeight*4 );
        memcpy( frameData.mDepthBuffer.data, mpDepthBuffer, mImageWidth*mImageHeight*sizeof( uint16_t ) );

        // Flip the images about the vertical
        cv::flip( frameData.mColorBuffer, frameData.mColorBuffer, 1 );
        cv::flip( frameData.mDepthBuffer, frameData.mDepthBuffer, 1 );

        // Save the frame
        mCalibrationImages.push_back( frameData );
        updateNumCalibrationImagesDisplay();
    }
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onBtnClearCalibrationImagesClicked()
{
    mCalibrationImages.clear();
    updateNumCalibrationImagesDisplay();
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onBtnCalculateCameraMatricesClicked()
{
    const uint32_t NUM_SAMPLE_POINTS = 100;
    const uint32_t MAX_NUM_SAMPLE_ATTEMPTS = 10000;

    if ( mCalibrationImages.size() > 0 )
    {
        // Get world points and image points for each calibration image
        std::vector< std::vector<cv::Point3f> > worldPointList;
        std::vector< std::vector<cv::Point2f> > depthImagePointList;
        std::vector< std::vector<cv::Point2f> > colorImagePointList;

        boost::random::uniform_int_distribution<> distX( 0, mImageWidth - 1 );
        boost::random::uniform_int_distribution<> distY( 0, mImageHeight - 1 );

        for ( uint32_t imageIdx = 0; imageIdx < mCalibrationImages.size(); imageIdx++ )
        {
            const FrameData& frameData = mCalibrationImages[ imageIdx ];

            std::vector<cv::Point3f> worldPoints;
            std::vector<cv::Point2f> depthImagePoints;
            std::vector<cv::Point2f> colorImagePoints;

            worldPoints.reserve( NUM_SAMPLE_POINTS );
            depthImagePoints.reserve( NUM_SAMPLE_POINTS );
            colorImagePoints.reserve( NUM_SAMPLE_POINTS );

            uint32_t numPointsTried = 0;
            while ( worldPoints.size() < NUM_SAMPLE_POINTS 
                && numPointsTried < MAX_NUM_SAMPLE_ATTEMPTS )
            {
                // Pick points at random from the depth image
                int x = distX( gRandomNumberGenerator );
                int y = distY( gRandomNumberGenerator );

                //int depthPixelIdx = y*mImageWidth + x;
                uint16_t depthValue = frameData.mDepthBuffer.at<uint16_t>( y, x );

                // Check that the depth value is valid
                if ( depthValue > 0 )
                {
                    // The depth value is valid. Now see if we have a corresponding color value
                    LONG colorX;
                    LONG colorY;
                    HRESULT hr = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                        IMAGE_RESOLUTION,
                        IMAGE_RESOLUTION,
                        NULL,
                        x, y, depthValue,
                        &colorX, &colorY );

                    if ( FAILED( hr ) )
                    {
                        printf( "Error: Failed to get color pixel coordinates\n" );
                        return;
                    }

                    // Check that the color coordinates are valid
                    if ( colorX >= 0 && colorX < (LONG)mImageWidth
                        && colorY >= 0 && colorY < (LONG)mImageHeight )
                    {
                        // We have a valid color so save a new set of points
                        Vector4 v = NuiTransformDepthImageToSkeleton( x, y, depthValue, IMAGE_RESOLUTION );
                        worldPoints.push_back( cv::Point3f( v.x, v.y, v.z ) );
                        depthImagePoints.push_back( cv::Point2f( x, y ) );
                        colorImagePoints.push_back( cv::Point2f( colorX, colorY ) );
                    }
                }

                numPointsTried++;
            }

            // Only proceed if we got enough points
            if ( worldPoints.size() < NUM_SAMPLE_POINTS )
            {
                printf( "Error: Unable to find %i sample points for image %i\n", NUM_SAMPLE_POINTS, imageIdx );
                return;
            }

            worldPointList.push_back( worldPoints );
            depthImagePointList.push_back( depthImagePoints );
            colorImagePointList.push_back( colorImagePoints );
        }

        // Use the OpenCV camera calibration routines to determine camera matrices
        mDepthCameraCalibrationMtx = cv::Mat( 3, 3, CV_32F );
        float *pM = (float*)mDepthCameraCalibrationMtx.data;
        pM[ 0 ] = 600.0f; pM[ 1 ] = 0.0f; pM[ 2 ] = mImageWidth/2.0f;
        pM[ 3 ] = 0.0f; pM[ 4 ] = 600.0f; pM[ 5 ] = mImageHeight/2.0f;
        pM[ 6 ] = 0.0f; pM[ 7 ] = 0.0f; pM[ 8 ] = 1.0f;

        mDepthCameraDistortionCoeffs = cv::Mat( 1, 5, CV_32F );
        mDepthCameraDistortionCoeffs.setTo( 0.0f );

        std::vector<cv::Mat> depthRotationVectors;
        std::vector<cv::Mat> depthTranslationVectors;
        
        float reprojectionError = cv::calibrateCamera(
            worldPointList, depthImagePointList, 
            cv::Size( mImageWidth, mImageHeight ), 
            mDepthCameraCalibrationMtx, mDepthCameraDistortionCoeffs, 
            depthRotationVectors, depthTranslationVectors, CV_CALIB_USE_INTRINSIC_GUESS );

        std::cout << "Depth Reprojection Error = " << reprojectionError << "\n";
        std::cout << "Depth Camera Calibration Matrix:\n";
        std::cout << mDepthCameraCalibrationMtx << "\n";

        mColorCameraCalibrationMtx = cv::Mat( 3, 3, CV_32F );
        pM = (float*)mColorCameraCalibrationMtx.data;
        pM[ 0 ] = 600.0f; pM[ 1 ] = 0.0f; pM[ 2 ] = mImageWidth/2.0f;
        pM[ 3 ] = 0.0f; pM[ 4 ] = 600.0f; pM[ 5 ] = mImageHeight/2.0f;
        pM[ 6 ] = 0.0f; pM[ 7 ] = 0.0f; pM[ 8 ] = 1.0f;

        mColorCameraDistortionCoeffs = cv::Mat( 1, 5, CV_32F );
        mColorCameraDistortionCoeffs.setTo( 0.0f );

        std::vector<cv::Mat> colorRotationVectors;
        std::vector<cv::Mat> colorTranslationVectors;

        reprojectionError = cv::calibrateCamera(
            worldPointList, colorImagePointList, 
            cv::Size( mImageWidth, mImageHeight ), 
            mColorCameraCalibrationMtx, mColorCameraDistortionCoeffs, 
            colorRotationVectors, colorTranslationVectors, CV_CALIB_USE_INTRINSIC_GUESS );

        std::cout << "Color Reprojection Error = " << reprojectionError << "\n";
        std::cout << "Color Camera Calibration Matrix:\n";
        std::cout << mColorCameraCalibrationMtx << "\n";

        // Use the stereo calibration routine to determine the relative poses of the cameras
        int flags = CV_CALIB_FIX_INTRINSIC;
        reprojectionError = cv::stereoCalibrate(
            worldPointList, depthImagePointList, colorImagePointList, 
            mDepthCameraCalibrationMtx, mDepthCameraDistortionCoeffs, 
            mColorCameraCalibrationMtx, mColorCameraDistortionCoeffs, 
            cv::Size( mImageWidth, mImageHeight ), 
            mDepthToColorCameraRotation, mDepthToColorCameraTranslation, 
            mDepthToColorCameraEssentialMatrix, mDepthToColorCameraFundamentalMatrix, 
            cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6 ), flags );

        std::cout << "Stereo Reprojection Error = " << reprojectionError << "\n";
        std::cout << "Relative Rotation:\n";
        std::cout << mDepthToColorCameraRotation << "\n";
        std::cout << "Relative Translation:\n";
        std::cout << mDepthToColorCameraTranslation << "\n";

        mbHaveValidCameraMatrices = true;
    }
}

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::onBtnSaveCameraMatricesClicked()
{
    if ( mbHaveValidCameraMatrices )
    {
        std::string baseDir = Utilities::getDataDir() + "/point_clouds/calibration_data";

        QString filename = QFileDialog::getSaveFileName( this,
            tr( "Save Calibration Data" ), baseDir.c_str(), tr("YAML Files (*.yaml)") );
        
        if ( !filename.isEmpty() )
        {
            cv::FileStorage dataFile( filename.toStdString(), cv::FileStorage::WRITE );
            
            dataFile << "DepthCameraCalibrationMtx" << mDepthCameraCalibrationMtx;
            dataFile << "DepthCameraDistortionCoeffs" << mDepthCameraDistortionCoeffs;
            dataFile << "ColorCameraCalibrationMtx" << mColorCameraCalibrationMtx;
            dataFile << "ColorCameraDistortionCoeffs" << mColorCameraDistortionCoeffs;
            dataFile << "DepthToColorCameraRotation" << mDepthToColorCameraRotation;
            dataFile << "DepthToColorCameraTranslation" << mDepthToColorCameraTranslation;
            dataFile << "DepthToColorCameraEssentialMatrix" << mDepthToColorCameraEssentialMatrix;
            dataFile << "DepthToColorCameraFundamentalMatrix" << mDepthToColorCameraFundamentalMatrix;

            dataFile.release();
        }
    }
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

	for ( int y = 0; y < (int)mImageHeight; y++ )
	{
		for ( int x = 0; x < (int)mImageWidth; x++ )
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

	/*if ( numPoints > 0 )
	{
		printf( "xScale mean %2.8f var %2.8f   yScale mean %2.8f var %2.8f   zScale mean %2.4f var %2.4f\n", 
			xScaleMean, xScaleM2/numPoints, yScaleMean, yScaleM2/numPoints, zScaleMean, zScaleM2/numPoints );
	}
	else
	{
		printf( "No depth points\n" );
	}*/

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

	// Mirror the image horizontally for display
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

//--------------------------------------------------------------------------------------------------
void WkgMainWindow::updateNumCalibrationImagesDisplay()
{
    this->lblNumCalibrationImages->setText( 
        QString( "Calibration Images: " ) + QString::number( mCalibrationImages.size() ) );
}

//------------------------------------------------------------------------------
void WkgMainWindow::saveDepthDataToSpcFile( const std::string& filename, 
    uint16_t* pDepthData, uint8_t* pDepthColorData, bool bBinary )
{
    FILE* pSpcFile = fopen( filename.c_str(), "wb" );

    if ( NULL != pSpcFile )
    {
        uint32_t width = mImageWidth;
        uint32_t height = mImageHeight;
        uint32_t focalLengthPixels = 2*NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS; // Doubled because we use a 640x480 resolution

        fprintf( pSpcFile, "WIDTH %u\n", width );
        fprintf( pSpcFile, "HEIGHT %u\n", height );
        fprintf( pSpcFile, "FOCAL_LENGTH_PIXELS %u\n", focalLengthPixels );
        fprintf( pSpcFile, "BINARY %i\n", (int32_t)bBinary );

        // Write out depth data
        fprintf( pSpcFile, "DEPTH_DATA\n" );
        std::vector<float> depthBuffer;
        depthBuffer.resize( width*height );
        float* pCurDepth = &depthBuffer[ 0 ];

        for ( uint32_t v = 0; v < height; v++ )
        {
            for ( uint32_t u = 0; u < width; u++ )
            {
                uint16_t depthValue = pDepthData[ v*width + u ];
                if ( depthValue > 0 )
                {
                    *pCurDepth = (float)depthValue / 1000.0f;
                }
                else
                {
                    *pCurDepth = std::numeric_limits<float>::quiet_NaN();
                }

                pCurDepth++;
            }
        }

        if ( bBinary )
        {
            fwrite( &depthBuffer[ 0 ], sizeof( float ), width*height, pSpcFile );
        }
        else
        {
            for ( uint32_t i = 0; i < depthBuffer.size(); i++ )
            {
                fprintf( pSpcFile, "%f\n", depthBuffer[ i ] );
            }
        }

        // Write out rgba data
        fprintf( pSpcFile, "RGBA_DATA\n" );

        if ( bBinary )
        {
            fwrite( pDepthColorData, sizeof( uint8_t ), 4*width*height, pSpcFile );
        }
        else
        {
            for ( uint32_t i = 0; i < width*height; i++ )
            {
                fprintf( pSpcFile, "%i %i %i %i\n",
                    pDepthColorData[ 4*i ], pDepthColorData[ 4*i + 1 ],
                    pDepthColorData[ 4*i + 1 ], pDepthColorData[ 4*i + 3 ] );
            }
        }

        fclose( pSpcFile );
    }
    else
    {
        throw std::runtime_error( "Unable to open file" );
    }
} 
