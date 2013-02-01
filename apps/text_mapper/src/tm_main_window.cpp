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
// File: tm_main_window.cpp
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkCellPicker.h>
#include <vtkProp3DCollection.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPropPicker.h>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "text_mapping/utilities.h"
#include "frame_dialog.h"
#include "tm_main_window.h"
#include <opencv2/core/eigen.hpp>
#include <sba/sba.h>

//--------------------------------------------------------------------------------------------------
static bool readVectorFromFileNode( cv::FileNode node, Eigen::Vector3f* pVectorOut )
{
    bool bVectorRead = false;

    if ( node.size() == 3
        && node[ 0 ].isReal() && node[ 1 ].isReal() && node[ 2 ].isReal() )
    {
        (*pVectorOut)[ 0 ] = (float)node[ 0 ];
        (*pVectorOut)[ 1 ] = (float)node[ 1 ];
        (*pVectorOut)[ 2 ] = (float)node[ 2 ];

        bVectorRead = true;
    }

    return bVectorRead;
}

//--------------------------------------------------------------------------------------------------
// TmMainWindow
//--------------------------------------------------------------------------------------------------
TmMainWindow::TmMainWindow()
    : mHighResImageViewDialog( this ),
      mKinectColorImageViewDialog( this ),
      mKinectDepthColorImageViewDialog( this )
{
    setupUi( this );

    mpFrameListModel = QSharedPointer<QStringListModel>( new QStringListModel() );
    this->listViewFrames->setModel( &(*mpFrameListModel) );

    // Store pointers for easy access to the image view dialogs
    mpImageViewDialogs.push_back( &mHighResImageViewDialog );
    mpImageViewDialogs.push_back( &mKinectColorImageViewDialog );
    mpImageViewDialogs.push_back( &mKinectDepthColorImageViewDialog );

    // Set up a renderer and connect it to QT
    mpRenderer = vtkRenderer::New();
    qvtkWidget->GetRenderWindow()->AddRenderer( mpRenderer );

    mpRenderer->SetBackground( 0.0, 0.0, 0.0 );
    mpRenderer->GetActiveCamera()->SetViewUp( 0.0, -1.0, 0.0 );
    mpRenderer->GetActiveCamera()->SetPosition( 2.0, -2.0, -2.0 );
    mpRenderer->GetActiveCamera()->SetFocalPoint( 0.0, 0.0, 2.0 );

    // Set up the pipeline to render a point cloud
    mpPointCloudSource = vtkSmartPointer<vtkPointCloudSource>::New();
    mpPointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpPointCloudActor = vtkSmartPointer<vtkActor>::New();

    mpPointCloudMapper->SetInputConnection( mpPointCloudSource->GetOutputPort() );
    mpPointCloudActor->SetMapper( mpPointCloudMapper );
    mpPointCloudActor->GetProperty()->SetPointSize( 3.0 );

    mpRenderer->AddActor( mpPointCloudActor );

    // Prepare to render pick point
    mpPickCubeSource = vtkSmartPointer<vtkCubeSource>::New();
    mpPickCubeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpPickCubeActor = vtkSmartPointer<vtkActor>::New();
    mpPickLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpPickLineActor = vtkSmartPointer<vtkActor>::New();

    mpPickCubeSource->SetXLength( 0.0025 );
    mpPickCubeSource->SetYLength( 0.0025 );
    mpPickCubeSource->SetZLength( 0.0025 );

    mpPickCubeMapper->SetInputConnection( mpPickCubeSource->GetOutputPort() );
    mpPickCubeActor->SetMapper( mpPickCubeMapper );
    mpPickCubeActor->GetProperty()->SetColor( 1.0, 0.0, 0.0 );
    mpPickCubeActor->SetVisibility( 0 );

    mpPickLineActor->SetMapper( mpPickLineMapper );
    mpPickLineActor->GetProperty()->SetLineWidth( 2 );
    mpPickLineActor->GetProperty()->SetColor( 1.0, 0.0, 0.0 );

    mpRenderer->AddActor( mpPickCubeActor );
    mpRenderer->AddActor( mpPickLineActor );

    // Prepare to render key point instances
    mpKeyPointInstancesSource = vtkSmartPointer<vtkKeyPointInstancesSource>::New();
    mpKeyPointInstancesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpKeyPointInstancesActor = vtkSmartPointer<vtkActor>::New();

    mpKeyPointInstancesMapper->SetInputConnection( mpKeyPointInstancesSource->GetOutputPort() );
    mpKeyPointInstancesActor->SetMapper( mpKeyPointInstancesMapper );
    mpKeyPointInstancesActor->GetProperty()->SetPointSize( 6 );

    mpRenderer->AddActor( mpKeyPointInstancesActor );

    // Prepare to render a box filter
    mpBoxFilterSource = vtkSmartPointer<vtkBoxFilterSource>::New();
    mpBoxFilterMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpBoxFilterActor = vtkSmartPointer<vtkActor>::New();

    mpBoxFilterMapper->SetInputConnection( mpBoxFilterSource->GetOutputPort() );
    mpBoxFilterActor->SetMapper( mpBoxFilterMapper );
    mpBoxFilterActor->GetProperty()->SetLineWidth( 3.0 );
    mpBoxFilterActor->SetVisibility( 0 );

    mpRenderer->AddActor( mpBoxFilterActor );

    // Load in object model
    std::string dataDir = Utilities::getDataDir();
    QString modelFilename = QString( dataDir.c_str() ) + "/models/carrs_crackers.obj";
    //QString modelFilename = QString( dataDir.c_str() ) + "/models/kinect/carrs_crackers/carrs_crackers.obj";
    loadObjModel( modelFilename );

    loadCameras();

    // Create a text map, source, and actor
    mpTextMapSource = vtkSmartPointer<vtkTextMapSource>::New();
    mpTextMapMapper = vtkPolyDataMapper::New();
    mpTextMapMapper->SetInput( mpTextMapSource->GetOutput() );

    mpTextMapActor = vtkActor::New();
    mpTextMapActor->SetMapper( mpTextMapMapper );

    // Load in a texture containing letters for the text map actor
    mpLettersJpegReader = vtkSmartPointer<vtkJPEGReader>::New();
    std::string fontsFilename = Utilities::getDataDir() + "/font/letters.jpg";
    mpLettersJpegReader->SetFileName( fontsFilename.c_str() );
    mpLettersJpegReader->Update();

    mpLettersTexture = vtkSmartPointer<vtkTexture>::New();
    mpLettersTexture->SetInputConnection( mpLettersJpegReader->GetOutputPort() );
    mpLettersTexture->InterpolateOn(); 
    
    // Apply it to the text map actor
    mpTextMapActor->SetTexture( mpLettersTexture ); 

    mpRenderer->AddActor( mpTextMapActor );

    // Set up a callback to handle mouse events in the main renderer
    mpDisplayEventCallback = vtkCallbackCommand::New();
    mpDisplayEventCallback->SetCallback( onInteractorEvent );
    mpDisplayEventCallback->SetClientData( this );

    qvtkWidget->GetInteractor()->AddObserver( vtkCommand::LeftButtonPressEvent, mpDisplayEventCallback );

    // Hook up signals
    connect( this->action_New, SIGNAL( triggered() ), this, SLOT( onNew() ) );
    connect( this->action_Open, SIGNAL( triggered() ), this, SLOT( onOpen() ) );
    connect( this->action_Save, SIGNAL( triggered() ), this, SLOT( onSave() ) );
    connect( this->action_Save_As, SIGNAL( triggered() ), this, SLOT( onSaveAs() ) );
    connect( this->action_Quit, SIGNAL( triggered() ), this, SLOT( close() ) );

    connect( this->listViewFrames->selectionModel(), 
             SIGNAL( currentChanged( const QModelIndex&, const QModelIndex& ) ), 
             this, 
             SLOT( onCurrentFrameChanged( const QModelIndex&, const QModelIndex& ) ) );
    
    connect( this->btnAddFrame, SIGNAL( clicked() ), this, SLOT( onBtnAddFrameClicked() ) );
    connect( this->btnEditFrame, SIGNAL( clicked() ), this, SLOT( onBtnEditFrameClicked() ) );
    connect( this->btnDeleteFrame, SIGNAL( clicked() ), this, SLOT( onBtnDeleteFrameClicked() ) );
    connect( this->btnDetectText, SIGNAL( clicked() ), this, SLOT( onBtnDetectTextClicked() ) );
    connect( this->btnAlignModelWithFrame, SIGNAL( clicked() ), this, SLOT( onBtnAlignModelWithFrameClicked() ) );
    connect( this->btnBuildModel, SIGNAL( clicked() ), this, SLOT( onBtnBuildModelClicked() ) );

    connect( this->btnLeft, SIGNAL( clicked() ), this, SLOT( onBtnLeftClicked() ) );
    connect( this->btnRight, SIGNAL( clicked() ), this, SLOT( onBtnRightClicked() ) );
    connect( this->btnUp, SIGNAL( clicked() ), this, SLOT( onBtnUpClicked() ) );
    connect( this->btnDown, SIGNAL( clicked() ), this, SLOT( onBtnDownClicked() ) );

    connect( this->checkShowModel, SIGNAL( clicked() ), this, SLOT( onCheckShowModelClicked() ) );
    connect( this->checkShowFrame, SIGNAL( clicked() ), this, SLOT( onCheckShowFrameClicked() ) );
    connect( this->checkShowFilter, SIGNAL( clicked() ), this, SLOT( onCheckShowFilterClicked() ) );

    connect( this->listWidgetKeyPoints, SIGNAL( currentRowChanged( int ) ),
             this, SLOT( onCurrentKeyPointRowChanged( int ) ) );
    connect( this->btnAddKeyPoint, SIGNAL( clicked() ), this, SLOT( onBtnAddKeyPointClicked() ) );
    connect( this->btnRemoveKeyPoint, SIGNAL( clicked() ), this, SLOT( onBtnRemoveKeyPointClicked() ) );
    connect( this->btnRemoveKeyPointModelInstance, SIGNAL( clicked() ), this, SLOT( onBtnRemoveKeyPointModelInstanceClicked() ) );
    connect( this->btnRemoveKeyPointFrameInstance, SIGNAL( clicked() ), this, SLOT( onBtnRemoveKeyPointFrameInstanceClicked() ) );
}

//--------------------------------------------------------------------------------------------------
TmMainWindow::~TmMainWindow()
{
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onNew()
{
    // Clear project data
    mFrames.clear();
    mKeyPoints.clear();
    refreshFrameList();
    refreshKeyPointList();
    refreshImageDisplays( NULL );

    // Update the 3D display
    qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onOpen()
{
    std::string projectDir = Utilities::getDataDir() + "/text_mapper_projects";
    std::string filename = QFileDialog::getOpenFileName( this,
         tr( "Open Project" ), projectDir.c_str(), tr("Project Files (*.yaml)") ).toStdString();

    if ( "" != filename )
    {
        loadProject( filename );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onSave()
{
    if ( mProjectFilename == "" )
    {
        onSaveAs();
    }
    else
    {
        saveProject( mProjectFilename );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onSaveAs()
{
    std::string projectDir = Utilities::getDataDir() + "/text_mapper_projects";

    QFileDialog saveDialog( this );
    saveDialog.setWindowTitle( tr( "Save Project" ) );
    saveDialog.setNameFilter( tr("Project Files (*.yaml)") );
    saveDialog.setDefaultSuffix( "yaml" );
    saveDialog.setDirectory( projectDir.c_str() );
    saveDialog.setAcceptMode( QFileDialog::AcceptSave );

    if ( saveDialog.exec() )
    {
        saveProject( saveDialog.selectedFiles()[ 0 ].toStdString() );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCurrentFrameChanged( const QModelIndex& current, const QModelIndex& previous )
{
    if ( current.isValid() )
    {
        int32_t curFrameIdx = current.row();
        if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
        {
            mFrames[ curFrameIdx ].tryToLoadImages( false );
            refreshImageDisplays( &mFrames[ curFrameIdx ] );
            refreshModelTransform();

            // Create a new text map for the frame
            //mpFrameTextMap = TextMap::Ptr( new TextMap() );
            //mpTextMapSource->SetTextMapPtr( mpFrameTextMap );

            // Set the transform for the text map
            mpTextMapSource->SetFrameTransform( getModelInFrameSpaceTransform() );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnAddFrameClicked()
{
    FrameData newFrameData;
    if ( FrameDialog::createNewFrame( &newFrameData ) )
    {
        refreshImageDisplays( &newFrameData );

        mFrames.push_back( newFrameData );
        refreshFrameList();

        // Select the last item that was added
        this->listViewFrames->setCurrentIndex( mpFrameListModel->index( mFrames.size() - 1 ) );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnEditFrameClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        FrameDialog::editFrame( &mFrames[ curFrameIdx ] );

        refreshImageDisplays( &mFrames[ curFrameIdx ] );
        refreshFrameList();
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnDeleteFrameClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        mFrames.erase( mFrames.begin() + curFrameIdx );
        refreshFrameList();

        // Update key point list to handle the frame being deleted
        for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
        {
            KeyPoint& keyPoint = mKeyPoints[ keyPointIdx ];

            std::vector<int32_t> instanceIndices = keyPoint.getKeyPointFrameInstanceIndices();
            std::sort( instanceIndices.begin(), instanceIndices.end() );

            for ( int32_t i = 0; i < (int32_t)instanceIndices.size(); i++ )
            {
                if ( i == curFrameIdx )
                {
                    // Remove the key point frame instance for the deleted frame
                    keyPoint.removeKeyPointFrameInstance( i );
                }
                else if ( i > curFrameIdx )
                {
                    // Move the key point frame instance down one
                    const KeyPointInstance* pFrameInstance = keyPoint.getKeyPointFrameInstance( i );
                    keyPoint.addKeyPointFrameInstance( i - 1, pFrameInstance->mPos );

                    keyPoint.removeKeyPointFrameInstance( i );
                }
            }
        }

        // Update the view to look at the current frame
        if ( curFrameIdx >= (int32_t)mFrames.size() )
        {
            curFrameIdx = (int32_t)mFrames.size() - 1;
        }

        if ( curFrameIdx >= 0 )
        {
            this->listViewFrames->setCurrentIndex( mpFrameListModel->index( curFrameIdx ) );
        }
        else
        {
            // No frames left
            refreshImageDisplays( NULL );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnDetectTextClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        printf( "Detecting text...\n" );

        Letter2DVector letters2D = detect_text( mFrames[ curFrameIdx ].mHighResImage, curFrameIdx );

        printf( "Found %u letter%s\n", (uint32_t)letters2D.size(), ( letters2D.size() == 1 ? "" : "s" ) );


        // Add the found letters to the text map
        //vtkCamera* pCurCamera = mpRenderer->GetActiveCamera();
        //mHighResCamera.setAsActiveCamera( mpRenderer );

        vtkSmartPointer<vtkCellPicker> pPicker = vtkSmartPointer<vtkCellPicker>::New();

        TextMap frameTextMap;
        for ( uint32_t letterIdx = 0; letterIdx < letters2D.size(); letterIdx++ )
        {
            printf( "Adding letter %u of %lu\n", letterIdx + 1, letters2D.size() );

            const Letter2D& letter2D = letters2D[ letterIdx ];
			Eigen::Vector3f topLeft;
			Eigen::Vector3f topRight;
			Eigen::Vector3f bottomLeft;

            Eigen::Vector3d lineStartPos;
            Eigen::Vector3d lineDir;
            mHighResCamera.getLineForPickPoint( letter2D.mTopLeft, &lineStartPos, &lineDir );

            float distanceToSurface = mFrames[ curFrameIdx ].mpKinectDepthPointCloud->pickSurface( 
                lineStartPos.cast<float>(), lineDir.cast<float>() );

            if ( distanceToSurface < 0.0 )
			{
				continue;
			}
			topLeft = lineStartPos.cast<float>() + distanceToSurface*lineDir.cast<float>();

            mHighResCamera.getLineForPickPoint( letter2D.mTopRight, &lineStartPos, &lineDir );

            distanceToSurface = mFrames[ curFrameIdx ].mpKinectDepthPointCloud->pickSurface( 
                lineStartPos.cast<float>(), lineDir.cast<float>() );

            if ( distanceToSurface < 0.0 )
			{
				continue;
			}
			topRight = lineStartPos.cast<float>() + distanceToSurface*lineDir.cast<float>();

            mHighResCamera.getLineForPickPoint( letter2D.mBottomLeft, &lineStartPos, &lineDir );

            distanceToSurface = mFrames[ curFrameIdx ].mpKinectDepthPointCloud->pickSurface( 
                lineStartPos.cast<float>(), lineDir.cast<float>() );

            if ( distanceToSurface < 0.0 )
			{
				continue;
			}
			bottomLeft = lineStartPos.cast<float>() + distanceToSurface*lineDir.cast<float>();

            mHighResCamera.getLineForPickPoint( letter2D.mBottomRight, &lineStartPos, &lineDir );

            distanceToSurface = mFrames[ curFrameIdx ].mpKinectDepthPointCloud->pickSurface( 
                lineStartPos.cast<float>(), lineDir.cast<float>() );

            if ( distanceToSurface < 0.0 )
			{
				continue;
			}


			Eigen::Vector3f centrePos = topLeft + (topRight - topLeft)/2.0 + (bottomLeft - topLeft)/2.0;
			Eigen::Vector3f axisY = topLeft - bottomLeft;
			float height = axisY.norm();
			axisY.normalize();
			Eigen::Vector3f axisX = topRight - topLeft;
			float width = axisX.norm();
			axisX.normalize();

			Eigen::Vector3f axisZ = axisX.cross( axisY );

			Letter letter;
			letter.mMtx = Eigen::Matrix4f::Identity();
			letter.mMtx.block<3,1>( 0, 0 ) = axisX;
			letter.mMtx.block<3,1>( 0, 1 ) = axisY;
			letter.mMtx.block<3,1>( 0, 2 ) = axisZ;
			letter.mMtx.block<3,1>( 0, 3 ) = centrePos;
			letter.mCharacter = letter2D.mCharacter;
			letter.mWidth = width;
			letter.mHeight = height;

			frameTextMap.addLetter( letter );
        }

        mFrames[ curFrameIdx ].setTextMap( frameTextMap );

        // Combine the text maps into one
        mpFrameTextMap = TextMap::Ptr( new TextMap() );

        for ( uint32_t frameIdx = 0; frameIdx < mFrames.size(); frameIdx++ )
        {
        	const FrameData& frame = mFrames[ frameIdx ];

        	if ( !frame.hasTextMap() )
        	{
        		continue;
        	}

        	Eigen::Matrix4f modelInFrameSpaceTransform = Eigen::Matrix4f::Identity();
        	if ( frame.isModelInFrameSpaceTransformSet() )
        	{
        		modelInFrameSpaceTransform = frame.getModelInFrameSpaceTransform();
        	}

        	Eigen::Matrix4f frameInModelSpaceTransform = modelInFrameSpaceTransform.inverse();

        	const TextMap& textMap = frame.getTextMap();
        	for ( uint32_t letterIdx = 0; letterIdx < textMap.getNumLetters(); letterIdx++ )
        	{
        		Letter letter = textMap.getLetter( letterIdx );

        		// Transform the letter into model space
        		letter.mMtx = frameInModelSpaceTransform*letter.mMtx;

        		// Add the letter to the combined text map
        		mpFrameTextMap->addLetter( letter );
        	}
        }


		mpTextMapSource->SetTextMapPtr( mpFrameTextMap );
		mpTextMapSource->SetFrameTransform( getModelInFrameSpaceTransform() );


        // Restore the camera
        //mpRenderer->SetActiveCamera( pCurCamera );

        qvtkWidget->update();

        
    }
    else
    {
        printf( "Error: No frame selected\n" );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnAlignModelWithFrameClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();

    if ( this->checkShowModel->isChecked()
        && curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        // Find the common set of key point instances shared by both the model and the current frame
        std::vector<Eigen::Vector3f> modelPoints;
        std::vector<Eigen::Vector3f> framePoints;

        modelPoints.reserve( mKeyPoints.size() );
        framePoints.reserve( mKeyPoints.size() );

        for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
        {
            const KeyPointInstance* pModelInstance = mKeyPoints[ keyPointIdx ].getKeyPointModelInstance();
            if ( NULL != pModelInstance )
            {
                const KeyPointInstance* pFrameInstance =
                    mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( curFrameIdx );

                if ( NULL != pFrameInstance )
                {
                    // This key point is represented by an instance both in the model, and in
                    // the frame
                    modelPoints.push_back( pModelInstance->mPos );
                    framePoints.push_back( pFrameInstance->mPos );
                }
            }
        }

        // Check that we've got enough points
        uint32_t numPoints = modelPoints.size();
        if ( numPoints < 3 )
        {
            QMessageBox::critical( NULL, "Error",
                "Not enough common key points. At least 3, non co-linear points are needed" );

            return;
        }

        // Find the transformation
        Eigen::Matrix3f rotationMtx;
        Eigen::Vector3f translation;
        float scale = 1.0;
        if ( !Utilities::findOptimumTransformation3D( &modelPoints[ 0 ], &framePoints[ 0 ],
            numPoints, &rotationMtx, &translation, &scale, true ) )
        {
            QMessageBox::critical( NULL, "Error",
                "Unable to find transform. This may be because of co-linear key points" );

            return;
        }

        // Store the transform
        Eigen::Matrix4f modelInFrameSpaceTransform = Eigen::Matrix4f::Identity();
        modelInFrameSpaceTransform.block<3,3>( 0, 0 ) = rotationMtx;
        modelInFrameSpaceTransform.block<3,1>( 0, 3 ) = translation;

        mFrames[ curFrameIdx ].setModelInFrameSpaceTransform( modelInFrameSpaceTransform );

        // Debug check scale...
        if ( Utilities::findOptimumTransformation3D( &modelPoints[ 0 ], &framePoints[ 0 ],
            numPoints, &rotationMtx, &translation, &scale ) )
        {
            printf( "Optimum scale would be %f\n", scale );
        }

        // Update the display
        refreshModelTransform();
        refreshKeyPointInstances();
        mpTextMapSource->SetFrameTransform( getModelInFrameSpaceTransform() );
        this->qvtkWidget->update();
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnBuildModelClicked()
{
	if ( mFrames.size() <= 0 )
	{
		return;
	}

	PointCloudWithPoseVector pointCloudsAndPoses;

	// Build the model around the first frame
	std::vector<Eigen::Vector3f> firstFrameKeyPoints;
	for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
	{
		const KeyPointInstance* pInstance = mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( 0 );
		if ( NULL != pInstance )
		{
			firstFrameKeyPoints.push_back( pInstance->mPos );
		}
	}

	// Load the frame if needed
	if ( NULL == mFrames[ 0 ].mpKinectDepthPointCloud )
	{
		if ( !mFrames[ 0 ].tryToLoadImages( true ) )
		{
			return;
		}
	}

	// Filter the frame if it has a box filter
	PointCloud::Ptr pFilteredCloud;
	if ( mFrames[ 0 ].hasBoxFilter() )
	{
	    pFilteredCloud = mFrames[ 0 ].mpKinectDepthPointCloud->filterWithBoxFilter(
	        mFrames[ 0 ].getBoxFilter() );
	}
	else
	{
	    pFilteredCloud = mFrames[ 0 ].mpKinectDepthPointCloud;
	}

	Eigen::Matrix4f combinedTransform = Eigen::Matrix4f::Identity();
	pointCloudsAndPoses.push_back( PointCloudWithPose(
		pFilteredCloud, combinedTransform, mFrames[ 0 ].mHighResImage, 0 ) );

	// Go through each of the remaining frames
	for ( uint32_t frameIdx = 1; frameIdx < mFrames.size(); frameIdx++ )
	{
		std::vector<Eigen::Vector3f> prevAlignmentKeyPoints;
		std::vector<Eigen::Vector3f> curAlignmentKeyPoints;

		// Get key points for this frame, along with the ones it has in common with the previous frame
		for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
		{
			const KeyPointInstance* pInstance = mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( frameIdx );
			if ( NULL != pInstance )
			{
				const KeyPointInstance* pOtherInstance = mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( frameIdx - 1 );
				if ( NULL != pOtherInstance )
				{
					prevAlignmentKeyPoints.push_back( pOtherInstance->mPos );
					curAlignmentKeyPoints.push_back( pInstance->mPos );
				}
			}
		}

		// Stop if we don't have enough common points to align the frames
		uint32_t numCommonPoints = prevAlignmentKeyPoints.size();
		if ( numCommonPoints < 3 )
		{
			printf( "Stopped at frame %u due to lack of key points\n", frameIdx );
			break;
		}

		// Load the frame if needed
		if ( NULL == mFrames[ frameIdx ].mpKinectDepthPointCloud )
		{
			if ( !mFrames[ frameIdx ].tryToLoadImages( true ) )
			{
				return;
			}
		}

		// Filter the frame if it has a box filter
        if ( mFrames[ frameIdx ].hasBoxFilter() )
        {
            pFilteredCloud = mFrames[ frameIdx ].mpKinectDepthPointCloud->filterWithBoxFilter(
                mFrames[ frameIdx ].getBoxFilter() );
        }
        else
        {
            pFilteredCloud = mFrames[ frameIdx ].mpKinectDepthPointCloud;
        }

		// Calculate the transform between frames
		Eigen::Matrix3f rotationMtx;
		Eigen::Vector3f translation;
		float scale = 1.0;
		if ( !Utilities::findOptimumTransformation3D( &curAlignmentKeyPoints[ 0 ],
			&prevAlignmentKeyPoints[ 0 ], numCommonPoints, &rotationMtx, &translation, &scale, true ) )
		{
			printf( "Stopped at frame %u as unable to calculate transform\n", frameIdx );
			break;
		}

		// Store the filtered point cloud and transform
		Eigen::Matrix4f curFrameInPrevFrameSpaceTransform = Eigen::Matrix4f::Identity();
		curFrameInPrevFrameSpaceTransform.block<3,3>( 0, 0 ) = rotationMtx;
		curFrameInPrevFrameSpaceTransform.block<3,1>( 0, 3 ) = translation;

		combinedTransform = combinedTransform*curFrameInPrevFrameSpaceTransform;
		pointCloudsAndPoses.push_back( PointCloudWithPose(
			pFilteredCloud, combinedTransform, mFrames[ frameIdx ].mHighResImage, frameIdx ) );
	}

	// Send a vector of point clouds and transforms to the model viewer dialog
	if ( this->checkUseLoopClosure->isChecked() )
	{
	    //refineAlignmentUsingSBA( pointCloudsAndPoses, &mHighResCamera );
	    refineAlignmentUsingLoopClosure( pointCloudsAndPoses );
	}

	mModelViewDialog.buildModel( pointCloudsAndPoses, &mHighResCamera );
	mModelViewDialog.show();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnLeftClicked()
{
    mHighResCamera.tweakLookAtPos( Eigen::Vector3d( -0.02, 0.0, 0.0 ) );
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnRightClicked()
{
    mHighResCamera.tweakLookAtPos( Eigen::Vector3d( 0.02, 0.0, 0.0 ) );
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnUpClicked()
{
    mHighResCamera.tweakLookAtPos( Eigen::Vector3d( 0.0, -0.02, 0.0 ) );
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnDownClicked()
{
    mHighResCamera.tweakLookAtPos( Eigen::Vector3d( 0.0, 0.02, 0.0 ) );
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCheckShowModelClicked()
{
	if ( this->checkShowModel->isChecked() )
	{
	    refreshModelTransform();
		mpModelActor->SetVisibility( 1 );
	}
	else
	{
		mpModelActor->SetVisibility( 0 );
	}

	refreshKeyPointInstances();
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCheckShowFrameClicked()
{
    if ( this->checkShowFrame->isChecked() )
    {
        mpPointCloudActor->SetVisibility( 1 );
    }
    else
    {
        mpPointCloudActor->SetVisibility( 0 );
    }

    refreshKeyPointInstances();
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCheckShowFilterClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        refreshPointCloudDisplay( &mFrames[ curFrameIdx ] );
    }
    else
    {
        refreshPointCloudDisplay( NULL );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCurrentKeyPointRowChanged( int currentRow )
{
    refreshKeyPointInstances();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnAddKeyPointClicked()
{
    mKeyPoints.push_back( KeyPoint() );
    refreshKeyPointList();

    // Select the last item that was added
    this->listWidgetKeyPoints->setCurrentRow( this->listWidgetKeyPoints->count() - 1 );
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnRemoveKeyPointClicked()
{
    int32_t curKeyPointIdx = this->listWidgetKeyPoints->currentRow();
    if ( curKeyPointIdx >= 0 && curKeyPointIdx < (int32_t)mKeyPoints.size() )
    {
        mKeyPoints.erase( mKeyPoints.begin() + curKeyPointIdx );
        refreshKeyPointList();
        refreshKeyPointInstances();
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnRemoveKeyPointModelInstanceClicked()
{
    int32_t curKeyPointIdx = this->listWidgetKeyPoints->currentRow();
    if ( curKeyPointIdx >= 0 && curKeyPointIdx < (int32_t)mKeyPoints.size() )
    {
        mKeyPoints[ curKeyPointIdx ].removeKeyPointModelInstance();
        refreshKeyPointInstances();
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnRemoveKeyPointFrameInstanceClicked()
{
    int32_t curKeyPointIdx = this->listWidgetKeyPoints->currentRow();
    if ( curKeyPointIdx >= 0 && curKeyPointIdx < (int32_t)mKeyPoints.size() )
    {
        int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
        if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
        {
            mKeyPoints[ curKeyPointIdx ].removeKeyPointFrameInstance( curFrameIdx );
            refreshKeyPointInstances();
        }
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::closeEvent( QCloseEvent* pEvent )
{
    for ( uint32_t dialogIdx = 0; dialogIdx < mpImageViewDialogs.size(); dialogIdx++ )
    {
        mpImageViewDialogs[ dialogIdx ]->close();
    }
    mModelViewDialog.close();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onInteractorEvent( vtkObject *caller, unsigned long eid,
                                      void* clientdata, void* calldata )
{
    TmMainWindow* pWin = (TmMainWindow*)clientdata;
    vtkRenderWindowInteractor* pInteractor = pWin->qvtkWidget->GetInteractor();

    switch ( eid )
    {
        case vtkCommand::LeftButtonPressEvent:
        {
            if ( pInteractor->GetControlKey() )
            {
                // Check to see if we have a key point and a frame selected
                int32_t curKeyPointIdx = pWin->listWidgetKeyPoints->currentRow();
                if ( curKeyPointIdx >= 0 && curKeyPointIdx < (int32_t)pWin->mKeyPoints.size() )
                {
                    // Now check to see if we've clicked on the object model
                    int* mousePos = pInteractor->GetEventPosition();

                    vtkSmartPointer<vtkPropPicker> pPicker = vtkSmartPointer<vtkPropPicker>::New();
                    pPicker->Pick( mousePos[ 0 ], mousePos[ 1 ], 0, pWin->mpRenderer );

                    if ( pPicker->GetActor() == pWin->mpModelActor )
                    {
                        double* pickPos = pPicker->GetPickPosition();
                        Eigen::Vector3f posInFrameSpace( pickPos[ 0 ], pickPos[ 1 ], pickPos[ 2 ] );

                        Eigen::Matrix4f modelInFrameSpaceTransform = pWin->getModelInFrameSpaceTransform();
                        Eigen::Matrix4f frameInModelSpaceTransform = modelInFrameSpaceTransform.inverse();

                        Eigen::Vector3f posInModelSpace =
                            frameInModelSpaceTransform.block<3,3>( 0, 0 )*posInFrameSpace
                            + frameInModelSpaceTransform.block<3,1>( 0, 3 );

                        pWin->mKeyPoints[ curKeyPointIdx ].addKeyPointModelInstance( posInModelSpace );
                        pWin->refreshKeyPointInstances();
                    }
                }
            }

            break;
        }
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::loadProject( const std::string& projectFilename )
{
    // Convert project filename to an absolute filename
    std::string absProjectFilename = Utilities::makeFilenameAbsoluteFromCWD( projectFilename );

    // Read in the new project
    cv::FileStorage fileStorage;
    fileStorage.open( absProjectFilename, cv::FileStorage::READ );

    // Read in frames
    std::vector<FrameData> newFrames;
    cv::FileNode framesNode = fileStorage[ "Frames" ];

    newFrames.reserve( framesNode.size() );
    for ( uint32_t frameIdx = 0; frameIdx < framesNode.size(); frameIdx++ )
    {
        cv::FileNode frameNode = framesNode[ frameIdx ];
        FrameData frameData;
        frameNode[ "HighResImage" ] >> frameData.mHighResImageFilename;
        frameData.mHighResImageFilename = Utilities::decodeRelativeFilename(
            absProjectFilename, frameData.mHighResImageFilename );
        frameNode[ "KinectColorImage" ] >> frameData.mKinectColorImageFilename;
        frameData.mKinectColorImageFilename = Utilities::decodeRelativeFilename(
            absProjectFilename, frameData.mKinectColorImageFilename );
        frameNode[ "KinectDepthPointCloud" ] >> frameData.mKinectDepthPointCloudFilename;
        frameData.mKinectDepthPointCloudFilename = Utilities::decodeRelativeFilename(
            absProjectFilename, frameData.mKinectDepthPointCloudFilename );

        BoxFilter boxFilter;
        cv::FileNode boxFilterNode = frameNode[ "BoxFilter" ];
        if ( BoxFilter::readBoxFilterFromFileStorage( boxFilterNode, &boxFilter ) )
        {
            frameData.setBoxFilter( boxFilter );
        }

        newFrames.push_back( frameData );
    }

    // Read in key points
    std::vector<KeyPoint> newKeyPoints;
    cv::FileNode keyPointsNode = fileStorage[ "KeyPoints" ];

    if ( !keyPointsNode.empty() )
    {
        newKeyPoints.reserve( keyPointsNode.size() );
        for ( uint32_t keyPointIdx = 0; keyPointIdx < keyPointsNode.size(); keyPointIdx++ )
        {
            cv::FileNode keyPointNode = keyPointsNode[ keyPointIdx ];

            KeyPoint keyPoint;

            // Look for frame instances
            cv::FileNode frameInstancesNode = keyPointNode[ "FrameInstances" ];
            if ( !frameInstancesNode.empty() )
            {
                for ( uint32_t frameInstanceIdx = 0; frameInstanceIdx < frameInstancesNode.size(); frameInstanceIdx++ )
                {
                    cv::FileNode frameInstanceNode = frameInstancesNode[ frameInstanceIdx ];

                    cv::FileNode frameIndexNode = frameInstanceNode[ "FrameIndex" ];
                    if ( !frameIndexNode.isInt() )
                    {
                        QMessageBox::critical( NULL, "Error", "Unable to read key point frame instance frame index" );
                        return;
                    }
                    int32_t frameIdx = (int32_t)frameIndexNode;

                    Eigen::Vector3f pos;
                    cv::FileNode posNode = frameInstanceNode[ "Pos" ];
                    if ( !readVectorFromFileNode( posNode, &pos ) )
                    {
                        QMessageBox::critical( NULL, "Error", "Unable to read key point frame instance position" );
                        return;
                    }

                    keyPoint.addKeyPointFrameInstance( frameIdx, pos );
                }
            }

            // Look for a model instance
            cv::FileNode modelInstanceNode = keyPointNode[ "ModelInstance" ];
            if ( !modelInstanceNode.empty() )
            {
                Eigen::Vector3f pos;
                cv::FileNode posNode = modelInstanceNode[ "Pos" ];
                if ( !readVectorFromFileNode( posNode, &pos ) )
                {
                    QMessageBox::critical( NULL, "Error", "Unable to read key point model instance position" );
                    return;
                }

                keyPoint.addKeyPointModelInstance( pos );
            }

            newKeyPoints.push_back( keyPoint );
        }
    }

    // Store the project filename
    mProjectFilename = absProjectFilename;

    // Clear out the existing project
    onNew();

    mFrames = newFrames;
    mKeyPoints = newKeyPoints;
    refreshFrameList();
    refreshKeyPointList();

    // Select the first frame
    this->listViewFrames->setCurrentIndex( mpFrameListModel->index( 0 ) );
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::saveProject( const std::string& projectFilename )
{
    // Convert project filename to an absolute filename
    std::string absProjectFilename = Utilities::makeFilenameAbsoluteFromCWD( projectFilename );

    // Write the data as a YAML file to memory
    cv::FileStorage fileStorage( absProjectFilename, cv::FileStorage::WRITE );

    // Write out the frames
    fileStorage << "Frames" << "[";
    for ( uint32_t frameIdx = 0; frameIdx < mFrames.size(); frameIdx++ )
    {
        fileStorage << "{";

        const FrameData& frameData = mFrames[ frameIdx ];
        fileStorage << "HighResImage"
            << Utilities::createRelativeFilename( absProjectFilename,
                Utilities::makeFilenameAbsoluteFromCWD( frameData.mHighResImageFilename ) );

        fileStorage << "KinectColorImage"
            << Utilities::createRelativeFilename( absProjectFilename,
                Utilities::makeFilenameAbsoluteFromCWD( frameData.mKinectColorImageFilename ) );

        fileStorage << "KinectDepthPointCloud"
            << Utilities::createRelativeFilename( absProjectFilename,
                Utilities::makeFilenameAbsoluteFromCWD( frameData.mKinectDepthPointCloudFilename ) );

        if ( frameData.hasBoxFilter() )
        {
            frameData.getBoxFilter().writeToFileStorage( fileStorage, "BoxFilter" );
        }

        fileStorage << "}";
    }

    fileStorage << "]";

    // Write out the key points
    fileStorage << "KeyPoints" << "[";
    for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
    {
        const KeyPoint& keyPoint = mKeyPoints[ keyPointIdx ];

        fileStorage << "{";

        // Store key point frame instances
        std::vector<int32_t> frameInstanceIndices = keyPoint.getKeyPointFrameInstanceIndices();
        if ( frameInstanceIndices.size() > 0 )
        {
            fileStorage << "FrameInstances" << "[";

            for ( uint32_t frameInstanceIdx = 0; frameInstanceIdx < frameInstanceIndices.size(); frameInstanceIdx++ )
            {
                int32_t frameIdx = frameInstanceIndices[ frameInstanceIdx ];
                const KeyPointInstance* pKeyPointInstance = keyPoint.getKeyPointFrameInstance( frameIdx );

                fileStorage << "{";

                fileStorage << "FrameIndex" << frameIdx;
                fileStorage << "Pos" << "[:";
                fileStorage << pKeyPointInstance->mPos[ 0 ];
                fileStorage << pKeyPointInstance->mPos[ 1 ];
                fileStorage << pKeyPointInstance->mPos[ 2 ];
                fileStorage << "]";

                fileStorage << "}";
            }

            fileStorage << "]";
        }

        // Store key point model instance if we have it
        if ( keyPoint.hasKeyPointModelInstanceIndices() )
        {
            const KeyPointInstance* pKeyPointInstance = keyPoint.getKeyPointModelInstance();

            fileStorage << "ModelInstance" << "{";

            fileStorage << "Pos" << "[:";
            fileStorage << pKeyPointInstance->mPos[ 0 ];
            fileStorage << pKeyPointInstance->mPos[ 1 ];
            fileStorage << pKeyPointInstance->mPos[ 2 ];
            fileStorage << "]";

            fileStorage << "}";
        }

        fileStorage << "}";
    }

    fileStorage << "]";

    mProjectFilename = absProjectFilename;
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::addKeyPointInstanceToFrameAtImagePos( const ImageViewDialog* pImageViewDialog, const QPointF& pickPoint )
{
    // Check to see if we have a key point and a frame selected
    int32_t curKeyPointIdx = this->listWidgetKeyPoints->currentRow();
    if ( curKeyPointIdx >= 0 && curKeyPointIdx < (int32_t)mKeyPoints.size() )
    {
        int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
        if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
        {
            // Now see if we're close to a point in the point cloud
            Eigen::Vector3f worldPos;
            bool bWorldPosFound = pickFromImage( pImageViewDialog, pickPoint, &worldPos, false );

            if ( bWorldPosFound )
            {
                mKeyPoints[ curKeyPointIdx ].addKeyPointFrameInstance( curFrameIdx, worldPos );
                refreshKeyPointInstances();
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
bool TmMainWindow::pickFromImage( const ImageViewDialog* pImageViewDialog, const QPointF& pickPoint,
                                  Eigen::Vector3f* pWorldPosOut, bool bDrawPickLine ) const
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx < 0 && curFrameIdx >= (int32_t)mFrames.size() )
    {
        // No frame selected...
        return false;
    }

    // Get the start point, and direction of the ray
    Eigen::Vector3d lineStartPos;
    Eigen::Vector3d lineDir;
    Eigen::Vector2d eigenPickPoint( pickPoint.x(), pickPoint.y() ); 

    if ( &mHighResImageViewDialog == pImageViewDialog )
    {
        mHighResCamera.getLineForPickPoint( eigenPickPoint, &lineStartPos, &lineDir );
    }
    else if ( &mKinectColorImageViewDialog == pImageViewDialog )
    {
        mKinectColorCamera.getLineForPickPoint( eigenPickPoint, &lineStartPos, &lineDir );
    }
    else if ( &mKinectDepthColorImageViewDialog == pImageViewDialog )
    {
        mKinectDepthCamera.getLineForPickPoint( eigenPickPoint, &lineStartPos, &lineDir );
    }
    else
    {
        // Invalid image view dialog...
        return false;
    }

    bool bSurfacePointFound = false;

    float distanceToSurface = mFrames[ curFrameIdx ].mpKinectDepthPointCloud->pickSurface(
        lineStartPos.cast<float>(), lineDir.cast<float>() );

    // Place the pick point
    if ( distanceToSurface >= 0.0 )
    {
        //std::cout << "Pos " << pickedWorldPos << std::endl;
        bSurfacePointFound = true;

        Eigen::Vector3f pickedWorldPos = lineStartPos.cast<float>() + lineDir.cast<float>()*distanceToSurface;

        if ( NULL != pWorldPosOut )
        {
            *pWorldPosOut = pickedWorldPos;
        }

        mpPickCubeActor->SetPosition( pickedWorldPos[ 0 ], pickedWorldPos[ 1 ], pickedWorldPos[ 2 ] );
    }

    if ( bDrawPickLine )
    {
        // Place a line to show the pick
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

        // Create the line
        points->InsertNextPoint( lineStartPos[ 0 ], lineStartPos[ 1 ], lineStartPos[ 2 ] );

        Eigen::Vector3d pickLineEnd = lineStartPos + 4.0*lineDir;
        points->InsertNextPoint( pickLineEnd[ 0 ], pickLineEnd[ 1 ], pickLineEnd[ 2 ] );

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId( 0, 0 );
        line->GetPointIds()->SetId( 1, 1 );

        // Store the line
        lines->InsertNextCell( line );

        // Create a polydata to store everything in
        vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

        // Add the points and lines to the dataset
        linesPolyData->SetPoints( points );
        linesPolyData->SetLines( lines );

        mpPickLineMapper->SetInput( linesPolyData );

        mpPickLineActor->SetVisibility( 1 );
        mpPickCubeActor->SetVisibility( 1 );

        this->qvtkWidget->update();
    }
    else
    {
        mpPickLineActor->SetVisibility( 0 );
        mpPickCubeActor->SetVisibility( 0 );

        this->qvtkWidget->update();
    }

    return bSurfacePointFound;
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::setBoxFilterFromImage( const ImageViewDialog* pImageViewDialog,
                                          const QRectF& filterRectangle  )
{
    // Check that we have a frame selected, and get the current point cloud
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx < 0 && curFrameIdx >= (int32_t)mFrames.size() )
    {
        // No frame selected...
        return;
    }
    PointCloud::Ptr pPointCloud = mFrames[ curFrameIdx ].mpKinectDepthPointCloud;

    // Get the camera for the image view dialog
    Camera* pCamera = NULL;
    if ( &mHighResImageViewDialog == pImageViewDialog )
    {
        pCamera = &mHighResCamera;
    }
    else if ( &mKinectColorImageViewDialog == pImageViewDialog )
    {
        pCamera = &mKinectColorCamera;
    }
    else if ( &mKinectDepthColorImageViewDialog == pImageViewDialog )
    {
        pCamera = &mKinectDepthCamera;
    }
    else
    {
        // Invalid image view dialog...
        return;
    }

    // Get information about the camera
    Eigen::Matrix4f cameraInWorldSpaceMtx = pCamera->getCameraInWorldSpaceMatrix().cast<float>();
    Eigen::Matrix3f cameraCalibMtx = pCamera->getCalibrationMatrix().cast<float>();

    const Eigen::Vector3f& cameraAxisX = cameraInWorldSpaceMtx.block<3,1>( 0, 0 );
    const Eigen::Vector3f& cameraAxisY = cameraInWorldSpaceMtx.block<3,1>( 0, 1 );
    const Eigen::Vector3f& cameraAxisZ = cameraInWorldSpaceMtx.block<3,1>( 0, 2 );
    const Eigen::Vector3f& cameraPos = cameraInWorldSpaceMtx.block<3,1>( 0, 3 );


    // Check every point against the z-plane, the closest that falls inside the filter rectangle
    // gives the near plane of the box filter
    bool bFoundClosestZ = false;
    float closestZ = FLT_MAX;

    for ( uint32_t pointIdx = 0; pointIdx < pPointCloud->getNumPoints(); pointIdx++ )
    {
        Eigen::Vector3f cameraToPoint = pPointCloud->getPointWorldPos( pointIdx ) - cameraPos;

        float pointZ = cameraToPoint.dot( cameraAxisZ );
        if ( pointZ > 0.0 && pointZ < closestZ )
        {
            // Check that the point projects into the filter rectangle
            float pointX = cameraToPoint.dot( cameraAxisX );
            float imageX = cameraCalibMtx( 0, 0 )*pointX/pointZ + cameraCalibMtx( 0, 2 );
            if ( imageX >= filterRectangle.left() && imageX <= filterRectangle.right() )
            {
                float pointY = cameraToPoint.dot( cameraAxisY );
                float imageY = cameraCalibMtx( 1, 1 )*pointY/pointZ + cameraCalibMtx( 1, 2 );
                if ( imageY >= filterRectangle.top() && imageY <= filterRectangle.bottom() )
                {
                    bFoundClosestZ = true;
                    closestZ = pointZ;
                }
            }
        }
    }

    // Set up the rest of the box filter.
    if ( bFoundClosestZ )
    {
        float SAFETY_MARGIN_Z = 0.01f;  // Extra space to try make sure we don't cut out points
                                        // due to errors in the relative camera poses

        float boxWidth = filterRectangle.width()*closestZ/cameraCalibMtx( 0, 0 );
        float boxHeight = filterRectangle.height()*closestZ/cameraCalibMtx( 1, 1 );
        float boxDepth = boxWidth + 2.0*SAFETY_MARGIN_Z;    // Add safety margin front and back

        // Cast a ray to find the centre of the box
        float distanceToBoxCentreZ = closestZ - SAFETY_MARGIN_Z + boxDepth/2.0f;
        Eigen::Vector3d rayStart;
        Eigen::Vector3d dirToBoxCentre;
        QPointF rectangleCentre = filterRectangle.center();
        pCamera->getLineForPickPoint(
            Eigen::Vector2d( rectangleCentre.x(), rectangleCentre.y() ),
            &rayStart, &dirToBoxCentre );

        float scaleZ = distanceToBoxCentreZ/(float)dirToBoxCentre[ 2 ];
        Eigen::Vector3d boxCentre = rayStart + dirToBoxCentre*scaleZ;

        Eigen::Matrix4f filterTransform = Eigen::Matrix4f::Identity();
        filterTransform.block<3,1>( 0, 0 ) = cameraAxisX;
        filterTransform.block<3,1>( 0, 1 ) = cameraAxisY;
        filterTransform.block<3,1>( 0, 2 ) = cameraAxisZ;
        filterTransform.block<3,1>( 0, 3 ) = boxCentre.cast<float>();

        // Build, set and show the filter
        BoxFilter boxFilter( filterTransform, Eigen::Vector3f( boxWidth, boxHeight, boxDepth ) );
        mFrames[ curFrameIdx ].setBoxFilter( boxFilter );
        this->checkShowFilter->setChecked( true );
        refreshPointCloudDisplay( &mFrames[ curFrameIdx ] );
    }
}

//--------------------------------------------------------------------------------------------------
Eigen::Matrix4f TmMainWindow::getModelInFrameSpaceTransform() const
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        if ( mFrames[ curFrameIdx ].isModelInFrameSpaceTransformSet() )
        {
            transform = mFrames[ curFrameIdx ].getModelInFrameSpaceTransform();
        }
    }

    return transform;
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refineAlignmentUsingSBA( PointCloudWithPoseVector& pointCloudsAndPoses,
                                       const Camera* pCamera )
{
    if ( pointCloudsAndPoses.size() <= 0 )
    {
        return;
    }

    // Set up an SBA system containing the initial guess for camera transforms and key point
    // positions
    sba::SysSBA* pSysSBA = new sba::SysSBA();

    // Create camera parameters.
    const Eigen::Matrix3d& calibMtx = pCamera->getCalibrationMatrix();
    frame_common::CamParams camParams;
    camParams.fx = calibMtx( 0, 0 ); // Focal length in x
    camParams.fy = calibMtx( 1, 1 ); // Focal length in y
    camParams.cx = calibMtx( 0, 2 ); // X position of principal point
    camParams.cy = calibMtx( 1, 2 ); // Y position of principal point
    camParams.tx = 0;   // Baseline (no baseline since this is monocular)

    // Add each of the cameras as nodes
    std::map<uint32_t, uint32_t> frameIdxToNodeIdxMap;

    for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
    {
        Eigen::Matrix4d cameraInWorldSpaceMtx =
            pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform.cast<double>()
            * pCamera->getCameraInWorldSpaceMatrix();


        Eigen::Quaterniond rot( cameraInWorldSpaceMtx.block<3,3>( 0, 0 ) );
        rot.normalize();
        Eigen::Vector4d trans = cameraInWorldSpaceMtx.block<4,1>( 0, 3 );

        frameIdxToNodeIdxMap[ pointCloudsAndPoses[ i ].mFrameIdx ] = i;

        bool bCameraFixed = (0 == i);   // Fix only the first node
        pSysSBA->addNode( trans, rot, camParams, bCameraFixed );
    }

    // Add each of the key points, both in world space, and in image space
    Eigen::Matrix4d worldInCameraSpaceMtx = pCamera->getCameraInWorldSpaceMatrix().inverse();
    int32_t imageWidth = pointCloudsAndPoses[ 0 ].mHighResImage.cols;
    int32_t imageHeight = pointCloudsAndPoses[ 0 ].mHighResImage.rows;

    std::map<uint32_t, int32_t> keyPointIdxToWorldPointIdxMap;

    for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
    {
        // Loop through all of the supplied frames to see if the key points are used there
        int32_t worldPointIdx = -1;

        for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
        {
            uint32_t frameIdx = pointCloudsAndPoses[ i ].mFrameIdx;
            const KeyPointInstance* pInstance =
                mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( frameIdx );
            if ( NULL != pInstance )
            {
                // Add a world position for the key point if needed
                if ( worldPointIdx < 0 )
                {
                    Eigen::Vector4d worldPos( pInstance->mPos[ 0 ],
                        pInstance->mPos[ 1 ], pInstance->mPos[ 2 ], 1.0 );
                    worldPos = pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform.cast<double>()*worldPos;

                    worldPointIdx = pSysSBA->addPoint( worldPos );
                    keyPointIdxToWorldPointIdxMap[ keyPointIdx ] = worldPointIdx;
                }

                // Now add the projected position of the key point for this frame
                Eigen::Vector3d posInCameraSpace =
                    worldInCameraSpaceMtx.block<3,3>( 0, 0 )*pInstance->mPos.cast<double>()
                    + worldInCameraSpaceMtx.block<3,1>( 0, 3 );
                Eigen::Vector2d projectedPosition(
                    calibMtx( 0, 0 )*posInCameraSpace[ 0 ]/posInCameraSpace[ 2 ] + calibMtx( 0, 2 ),
                    calibMtx( 1, 1 )*posInCameraSpace[ 1 ]/posInCameraSpace[ 2 ] + calibMtx( 1, 2 ) );

                if ( projectedPosition[ 0 ] >= 0 && projectedPosition[ 0 ] < imageWidth
					&& projectedPosition[ 1 ] >= 0 && projectedPosition[ 1 ] < imageHeight )
				{

                	int32_t nodeIdx = frameIdxToNodeIdxMap[ frameIdx ];

                	Eigen::Vector2d actualProj;
					pSysSBA->nodes[ nodeIdx ].setProjection();
					pSysSBA->nodes[ nodeIdx ].project2im( actualProj, pSysSBA->tracks[ worldPointIdx ].point );

					double error = (actualProj - projectedPosition).norm();
					if ( error > 10.0f )
					{
						printf( "Key point %u has more than 10 pixels error in frame %u\n", keyPointIdx, frameIdx );
					}

                    pSysSBA->addMonoProj( nodeIdx, worldPointIdx, projectedPosition );
                }
            }
        }
    }

    pSysSBA->removeBad( 10.0 );

    // Run bundle adjustment
    printf( "Preparing for SBA - Cameras (nodes): %d, Points: %d\n",
        (int32_t)pSysSBA->nodes.size(), (int32_t)pSysSBA->tracks.size() );

    int32_t numPoints = (int32_t)pSysSBA->tracks.size();
    printf( "Bad projs (> 10 pix): %d, Cost without: %f\n",
        (int)pSysSBA->countBad(10.0), sqrt(pSysSBA->calcCost(10.0)/numPoints) );
    printf( "Bad projs (> 5 pix): %d, Cost without: %f\n",
        (int)pSysSBA->countBad(5.0), sqrt(pSysSBA->calcCost(5.0)/numPoints)) ;
    printf( "Bad projs (> 2 pix): %d, Cost without: %f\n",
        (int)pSysSBA->countBad(2.0), sqrt(pSysSBA->calcCost(2.0)/numPoints) );

    pSysSBA->verbose = 1;
    pSysSBA->doSBA(10, 1e-3, SBA_DENSE_CHOLESKY); //SBA_SPARSE_CHOLESKY);

    printf( "Done SBA\n" );

        printf( "Bad projs (> 10 pix): %d, Cost without: %f\n",
            (int)pSysSBA->countBad(10.0), sqrt(pSysSBA->calcCost(10.0)/numPoints) );
        printf( "Bad projs (> 5 pix): %d, Cost without: %f\n",
            (int)pSysSBA->countBad(5.0), sqrt(pSysSBA->calcCost(5.0)/numPoints)) ;
        printf( "Bad projs (> 2 pix): %d, Cost without: %f\n",
            (int)pSysSBA->countBad(2.0), sqrt(pSysSBA->calcCost(2.0)/numPoints) );

    // Update the point cloud poses
    // Start from node 1, we don't need to do node 0 as it's fixed
	for ( uint32_t i = 1; i < pointCloudsAndPoses.size(); i++ )
	{
		Eigen::Matrix4d cameraInWorldSpaceMtx = Eigen::Matrix4d::Identity();

		cameraInWorldSpaceMtx.block<3,3>( 0, 0 ) = pSysSBA->nodes[ i ].qrot.matrix();
		cameraInWorldSpaceMtx.block<4,1>( 0, 3 ) = pSysSBA->nodes[ i ].trans;

		Eigen::Matrix4d newTransform = cameraInWorldSpaceMtx*worldInCameraSpaceMtx;
		pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform = newTransform.cast<float>();

		// Update the key point positions
		for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
		{
			// Loop through all of the supplied frames to see if the key points are used there
			int32_t worldPointIdx = keyPointIdxToWorldPointIdxMap[ keyPointIdx ];

			for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
			{
				uint32_t frameIdx = pointCloudsAndPoses[ i ].mFrameIdx;
				const KeyPointInstance* pInstance =
					mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( frameIdx );
				if ( NULL != pInstance )
				{
					// Get the modified world position for the key point
					Eigen::Vector4d newWorldPos = pSysSBA->tracks[ worldPointIdx ].point;

					newWorldPos = pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform.inverse().cast<double>()*newWorldPos;
					Eigen::Vector3f keyPointPos = newWorldPos.head<3>().cast<float>();
					mKeyPoints[ keyPointIdx ].addKeyPointFrameInstance( frameIdx, keyPointPos );
				}
			}
		}
	}

	delete pSysSBA;
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refineAlignmentUsingLoopClosure( PointCloudWithPoseVector& pointCloudsAndPoses ) const
{
    // Look for key points that link the first and last frame
    if ( pointCloudsAndPoses.size() < 3 )
    {
        printf( "Not enough frames for loop closure\n" );
        return;
    }

    uint32_t firstFrameIdx = pointCloudsAndPoses[ 0 ].mFrameIdx;
    uint32_t lastFrameIdx = pointCloudsAndPoses[ pointCloudsAndPoses.size() - 1 ].mFrameIdx;

    std::vector<Eigen::Vector3f> firstFrameKeyPoints;
    std::vector<Eigen::Vector3f> lastFrameKeyPoints;
    const Eigen::Matrix4f& lastFrameInFirstFrameTransform =
        pointCloudsAndPoses[ lastFrameIdx ].mPointCloudInWorldSpaceTransform;

    for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
    {
        const KeyPointInstance* pInstance =
            mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( firstFrameIdx );
        if ( NULL != pInstance )
        {
            const KeyPointInstance* pOtherInstance =
                mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( lastFrameIdx );
            if ( NULL != pOtherInstance )
            {
                firstFrameKeyPoints.push_back( pInstance->mPos );

                lastFrameKeyPoints.push_back(
                    lastFrameInFirstFrameTransform.block<3,3>( 0, 0 )*pOtherInstance->mPos
                    + lastFrameInFirstFrameTransform.block<3,1>( 0, 3 ) );
            }
        }
    }

    // Check that we have enough key points to perform loop closure
    uint32_t numCommonPoints = firstFrameKeyPoints.size();
    if ( numCommonPoints < 3 )
    {
        printf( "Not enough common key points for loop closure\n" );
        return;
    }

    // Look for the transform which will close the loop
    Eigen::Matrix3f rotationMtx;
    Eigen::Vector3f translation;
    float scale = 1.0;
    if ( !Utilities::findOptimumTransformation3D( &lastFrameKeyPoints[ 0 ],
        &firstFrameKeyPoints[ 0 ], numCommonPoints, &rotationMtx, &translation, &scale, true ) )
    {
        printf( "Unable to calculate transform for loop closure\n" );
        return;
    }

    Eigen::Vector3f extraRotation = rotationMtx.eulerAngles( 0, 1, 2 );
    printf( "Extra rotation %f %f %f\n",
        Utilities::radToDeg( extraRotation[ 0 ] ),
        Utilities::radToDeg( extraRotation[ 1 ] ),
        Utilities::radToDeg( extraRotation[ 2 ] ) );

    // Spread this transform around the loop by breaking it up into little pieces
    Eigen::Quaternionf initialRot( Eigen::Matrix3f::Identity() );
    Eigen::Quaternionf finalRot( rotationMtx );

    for ( uint32_t i = 1; i < pointCloudsAndPoses.size(); i++ )
    {
        Eigen::Matrix4f extraTransform = Eigen::Matrix4f::Identity();

        float progress = (float)i/(float)pointCloudsAndPoses.size();
        //float progress = 1.0f/(float)pointCloudsAndPoses.size();
        Eigen::Quaternionf extraRot = initialRot.slerp( progress, finalRot );
        Eigen::Vector3f extraTrans = progress*translation;

        extraTransform.block<3,3>( 0, 0 ) = extraRot.matrix();
        extraTransform.block<3,1>( 0, 3 ) = extraTrans;

        pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform =
            extraTransform * pointCloudsAndPoses[ i ].mPointCloudInWorldSpaceTransform;
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshFrameList()
{
    // Create a list of strings and thumbnails representing the frames
    QStringList list;
    for ( uint32_t frameIdx = 0; frameIdx < mFrames.size(); frameIdx++ )
    {
        QString frameName = QString( "frame " ) + QString::number( frameIdx );
        list << frameName;
    }
            
    mpFrameListModel->setStringList( list );
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshKeyPointList()
{
    this->listWidgetKeyPoints->clear();

    // Add the key points to the list widget
    for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
    {
        QListWidgetItem* pNewItem = new QListWidgetItem();
        pNewItem->setText( QString( "KeyPoint " ) + QString::number( keyPointIdx ) );
        this->listWidgetKeyPoints->insertItem( keyPointIdx, pNewItem );
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshPointCloudDisplay( const FrameData* pFrameData )
{
    if ( NULL == pFrameData )
    {
        // Clear the frame display
        mpPointCloudActor->SetVisibility( 0 );
        mpBoxFilterActor->SetVisibility( 0 );
    }
    else
    {
        if ( this->checkShowFilter->isChecked()
            && pFrameData->hasBoxFilter() )
        {
            mpFilteredPointCloud = pFrameData->mpKinectDepthPointCloud->filterWithBoxFilter(
                pFrameData->getBoxFilter() );
            mpPointCloudSource->SetPointCloudPtr( mpFilteredPointCloud );

            mpBoxFilterSource->SetBoxFilter( pFrameData->getBoxFilter() );
            mpBoxFilterActor->SetVisibility( 1 );
        }
        else
        {
            mpPointCloudSource->SetPointCloudPtr( pFrameData->mpKinectDepthPointCloud );
            mpBoxFilterActor->SetVisibility( 0 );
        }

        mpPointCloudActor->SetVisibility( 1 );
    }

    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshImageDisplays( const FrameData* pFrameData )
{
    if ( NULL == pFrameData )
    {
        // Hide image view dialogs
        for ( uint32_t dialogIdx = 0; dialogIdx < mpImageViewDialogs.size(); dialogIdx++ )
        {
            mpImageViewDialogs[ dialogIdx ]->hide();
        }
    }
    else
    {
        mHighResImageViewDialog.setImage( pFrameData->mHighResImage );
        mHighResImageViewDialog.setWindowTitle( "High Res Image" );
        mHighResImageViewDialog.show();
        mKinectColorImageViewDialog.setImage( pFrameData->mKinectColorImage );
        mKinectColorImageViewDialog.setWindowTitle( "Kinect Color Image" );
        mKinectColorImageViewDialog.show();
        mKinectDepthColorImageViewDialog.setImage( pFrameData->mpKinectDepthPointCloud->getImage() );
        mKinectDepthColorImageViewDialog.setWindowTitle( "Kinect Depth Camera Color Image" );
        mKinectDepthColorImageViewDialog.show();
    }

    refreshPointCloudDisplay( pFrameData );
    refreshKeyPointInstances();
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshKeyPointInstances()
{
    // Create a list of the key point instances to display
    std::vector<KeyPointInstanceData> frameKeyPointInstances;

    // TODO: Tidy the handling of image view dialogs up
    const uint32_t NUM_IMAGE_VIEW_DIALOGS = 3;
    std::vector<KeyPointInstanceData> imageKeyPointInstances[ 3 ];

    Eigen::Matrix4d worldInCameraSpaceMatrices[ 3 ] =
    {
		mHighResCamera.getCameraInWorldSpaceMatrix().inverse(),
		mKinectColorCamera.getCameraInWorldSpaceMatrix().inverse(),
		mKinectDepthCamera.getCameraInWorldSpaceMatrix().inverse()
    };

    Eigen::Matrix3d calibrationMatrices[ 3 ] =
	{
		mHighResCamera.getCalibrationMatrix(),
		mKinectColorCamera.getCalibrationMatrix(),
		mKinectDepthCamera.getCalibrationMatrix()
	};

	ImageViewDialog* imageViewDialogs[ 3 ] =
	{
		&mHighResImageViewDialog,
		&mKinectColorImageViewDialog,
		&mKinectDepthColorImageViewDialog
	};

    int32_t selectedKeyPointIdx = this->listWidgetKeyPoints->currentRow();

    // Add frame instances first
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
        {
            const KeyPointInstance* pInstance = mKeyPoints[ keyPointIdx ].getKeyPointFrameInstance( curFrameIdx );
            if ( NULL != pInstance )
            {
            	KeyPointInstanceData instanceData;

                if ( (int32_t)keyPointIdx == selectedKeyPointIdx )
                {
                    // Yellow for selected frame instance
                	instanceData = KeyPointInstanceData( *pInstance, 255, 255, 0 );
                }
                else
                {
                    // Green for normal frame instance
                	instanceData = KeyPointInstanceData( *pInstance, 0, 255, 0 );
                }

                // Project the key point into each of the image displays
                for ( uint32_t i = 0; i < NUM_IMAGE_VIEW_DIALOGS; i++ )
                {
                	const Eigen::Matrix3d& calibMtx = calibrationMatrices[ i ];

                	Eigen::Vector3d posInCameraSpace =
						worldInCameraSpaceMatrices[ i ].block<3,3>( 0, 0 )*instanceData.mKeyPointInstance.mPos.cast<double>()
						+ worldInCameraSpaceMatrices[ i ].block<3,1>( 0, 3 );
                	instanceData.mProjectedPosition = Eigen::Vector2f(
						calibMtx( 0, 0 )*posInCameraSpace[ 0 ]/posInCameraSpace[ 2 ] + calibMtx( 0, 2 ),
						calibMtx( 1, 1 )*posInCameraSpace[ 1 ]/posInCameraSpace[ 2 ] + calibMtx( 1, 2 ) );

                	imageKeyPointInstances[ i ].push_back( instanceData );
                }

                frameKeyPointInstances.push_back( instanceData );
            }
        }
    }

    // Add in model instances if needed
    if ( this->checkShowModel->isChecked() )
    {
        Eigen::Matrix4f modelInFrameSpaceTransform = getModelInFrameSpaceTransform();

        for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
        {
            const KeyPointInstance* pInstance = mKeyPoints[ keyPointIdx ].getKeyPointModelInstance();
            if ( NULL != pInstance )
            {
                KeyPointInstance transformedInstance = *pInstance;
                transformedInstance.mPos =
                    modelInFrameSpaceTransform.block<3,3>( 0, 0 )*transformedInstance.mPos
                    + modelInFrameSpaceTransform.block<3,1>( 0, 3 );

                if ( (int32_t)keyPointIdx == selectedKeyPointIdx )
                {
                    // Yellow for selected model instance
                    frameKeyPointInstances.push_back( KeyPointInstanceData( transformedInstance, 255, 255, 0 ) );
                }
                else
                {
                    // Green for normal model instance
                	frameKeyPointInstances.push_back( KeyPointInstanceData( transformedInstance, 0, 255, 0 ) );
                }
            }
        }
    }

    // Display the key point instances
    for ( uint32_t i = 0; i < NUM_IMAGE_VIEW_DIALOGS; i++ )
	{
    	imageViewDialogs[ i ]->setKeyPointInstancesToDisplay( imageKeyPointInstances[ i ] );
	}

    mpKeyPointInstancesSource->SetKeyPointInstances( frameKeyPointInstances );
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshModelTransform()
{
    Eigen::Matrix4f modelTransform = getModelInFrameSpaceTransform();

    // Break the rotation matrix down into angles Z, X, Y. The order in which VTK will apply them
    Eigen::Vector3f angles = modelTransform.block<3,3>( 0, 0 ).eulerAngles( 2, 0, 1 );

    // Now pass the angles as degrees to VTK
    float degreesX = Utilities::radToDeg( angles[ 1 ] );
    float degreesY = Utilities::radToDeg( angles[ 2 ] );
    float degreesZ = Utilities::radToDeg( angles[ 0 ] );
    mpModelActor->SetOrientation( degreesX, degreesY, degreesZ );

    const Eigen::Vector3f& pos = modelTransform.block<3,1>( 0, 3 );
    mpModelActor->SetPosition( pos[ 0 ], pos[ 1 ], pos[ 2 ] );
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::loadCameras()
{
    mKinectDepthCamera.setImageSize( 640.0f, 480.0f );
    mKinectColorCamera.setImageSize( 640.0f, 480.0f );
    mHighResCamera.setImageSize( 2272.0f, 1704.0f );

    // TODO: Move away from hard coded cameras
    std::string dataDir = Utilities::getDataDir();
    //std::string kinectCalibrationFilename = dataDir + "/point_clouds/calibration_data/windows_kinect.yaml";
    //std::string highResCalibrationFilename = dataDir + "/calibration_images/canon_zoom4_cameraMatrix_2.yml";
    //std::string highResPoseFilename = dataDir + "/calibration_images/stereo_calibration.yml";

    //std::string kinectCalibrationFilename = dataDir + "/calibration_images/Simulated/kinect_calib.yaml";
    //std::string highResCalibrationFilename = dataDir + "/calibration_images/Simulated/high_res_calib.yaml";
    //std::string highResCalibrationFilename = dataDir + "/calibration_images/Simulated/HighResRGB_cameraMatrix.yml";
    //std::string highResPoseFilename = dataDir + "/calibration_images/Simulated/colour_stereo_calib.yaml";
    //std::string highResPoseFilename = dataDir + "/calibration_images/Simulated/KinectRGB_to_HighResRGB_calib.yml";

    std::string kinectCalibrationFilename = dataDir + "/point_clouds/calibration_data/windows_kinect.yaml";
    std::string highResCalibrationFilename = dataDir + "/calibration_images/Canon_Zoom4/CanonZoom4_cameraMatrix.yml";
    //std::string highResPoseFilename = dataDir + "/calibration_images/Kinect_Canon_Stereo/KinectRGB_to_CanonZoom4_calib.yml";
    std::string highResPoseFilename = dataDir + "/calibration_images/Kinect_Canon_Stereo/KinectDepth_to_CanonZoom4_calib.yml";
    //std::string highResPoseFilename = dataDir + "/calibration_images/Kinect_Canon_Stereo/KinectDepth_to_BertCanonZoom4_calib.yml";

    // Load in the Kinect calibration file
    cv::FileStorage fileStorage;
    fileStorage.open( kinectCalibrationFilename, cv::FileStorage::READ );

    // Read out matrices for the color and depth camera on the Kinect
    cv::Mat depthCameraCalibrationMatrix;
    cv::Mat colorCameraCalibrationMatrix;

    fileStorage[ "DepthCameraCalibrationMtx" ] >> depthCameraCalibrationMatrix;
    fileStorage[ "ColorCameraCalibrationMtx" ] >> colorCameraCalibrationMatrix;

    cv::Mat depthToColorCameraRotationMatrix;
    cv::Mat depthToColorCameraTranslationVector;

    fileStorage[ "DepthToColorCameraRotation" ] >> depthToColorCameraRotationMatrix;
    fileStorage[ "DepthToColorCameraTranslation" ] >> depthToColorCameraTranslationVector;

    fileStorage.release();

    // Now load in calibration files for the high resolution camera
    fileStorage.open( highResCalibrationFilename, cv::FileStorage::READ );
    cv::Mat highResCameraCalibrationMatrix;
    fileStorage[ "cameraMatrix" ] >> highResCameraCalibrationMatrix;
    fileStorage.release();

    std::cout << "High Res Mtx..." << std::endl;
    std::cout << highResCameraCalibrationMatrix << std::endl;

    fileStorage.open( highResPoseFilename, cv::FileStorage::READ );
    cv::Mat depthToHighResCameraRotationMatrix;
    cv::Mat depthToHighResCameraTranslationVector;
    fileStorage[ "R" ] >> depthToHighResCameraRotationMatrix;
    fileStorage[ "T" ] >> depthToHighResCameraTranslationVector;
    fileStorage.release();

    // Set up the cameras

    // Depth
    mKinectDepthCamera.setCameraInWorldSpaceMatrix( Eigen::Matrix4d::Identity() );

    Eigen::Matrix3d eigen3x3;
    cv2eigen( depthCameraCalibrationMatrix, eigen3x3 );

    mKinectDepthCamera.setCalibrationMatrix( eigen3x3 );

    // Color
    Eigen::Matrix4d kinectColorCameraInWorldSpaceMatrix = Eigen::Matrix4d::Identity();

    cv2eigen( depthToColorCameraRotationMatrix, eigen3x3 );

    kinectColorCameraInWorldSpaceMatrix.block<3,3>( 0, 0 ) = eigen3x3;
    kinectColorCameraInWorldSpaceMatrix.block<3,1>( 0, 3 ) = 
        Eigen::Map<Eigen::Vector3d>( (double*)depthToColorCameraTranslationVector.data, 3, 1 );

    // HACK: Flipping about the y-axis. This should be done back in the kinect grabber
    //kinectColorCameraInWorldSpaceMatrix.block<1,4>( 0, 0 ) = -kinectColorCameraInWorldSpaceMatrix.block<1,4>( 0, 0 );

    std::cout << depthToColorCameraRotationMatrix << std::endl;
    std::cout << kinectColorCameraInWorldSpaceMatrix << std::endl;

    mKinectColorCamera.setCameraInWorldSpaceMatrix( kinectColorCameraInWorldSpaceMatrix );

    cv2eigen( colorCameraCalibrationMatrix, eigen3x3 );
    mKinectColorCamera.setCalibrationMatrix( eigen3x3 );

    // High Resolution
    Eigen::Matrix4d highResCameraInDepthCameraSpaceMatrix = Eigen::Matrix4d::Identity();

    cv2eigen( depthToHighResCameraRotationMatrix, eigen3x3 );
    highResCameraInDepthCameraSpaceMatrix.block<3,3>( 0, 0 ) = eigen3x3;
    highResCameraInDepthCameraSpaceMatrix.block<3,1>( 0, 3 ) = 
        Eigen::Map<Eigen::Vector3d>( (double*)depthToHighResCameraTranslationVector.data, 3, 1 );
    
    mHighResCamera.setCameraInWorldSpaceMatrix( highResCameraInDepthCameraSpaceMatrix );

    cv2eigen( highResCameraCalibrationMatrix, eigen3x3 );
    mHighResCamera.setCalibrationMatrix( eigen3x3 );
    //mHighResCamera.setClipPlanes( 0.55, 2.0 );

    //mHighResCamera.tweakLookAtPos( HIGH_RES_CAMERA_OFFSET );

    mKinectDepthCamera.setClipPlanes( 0.2, 2.0 );
    mKinectColorCamera.setClipPlanes( 0.2, 2.0 );
    mHighResCamera.setClipPlanes( 0.2, 2.0 );

    // Display them in the VTK renderer
    mKinectDepthCamera.showInRenderer( mpRenderer );
    mKinectDepthCamera.setColor( 0, 255, 0 );
    mKinectColorCamera.showInRenderer( mpRenderer );
    mHighResCamera.showInRenderer( mpRenderer );
    mHighResCamera.setColor( 255, 0, 255 );

    //mHighResCamera.addPickPoint( Eigen::Vector2d( 1057.0, 124.0 ) );
    //mHighResCamera.addPickPoint( Eigen::Vector2d( 1523.0, 176.0 ) );
    //mHighResCamera.addPickPoint( Eigen::Vector2d( 1025.0, 1556.0 ) );
    //mHighResCamera.addPickPoint( Eigen::Vector2d( 1496.0, 1562.0 ) );
    //mKinectDepthCamera.addPickPoint( Eigen::Vector2d( 480.0, 360.0 ) );

    qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::loadObjModel( QString filename )
{
    if ( !filename.isEmpty() )
    {
        if ( NULL != mpObjReader )
        {
            // Object has already been setup so just update the model
            mpObjReader->SetFileName( filename.toLatin1() );
            mpObjReader->Update();
        }
        else
        {
            // Prepare to read in Obj files
            mpObjReader = vtkSmartPointer<vtkOBJReader>::New();
            mpObjReader->SetFileName( filename.toLatin1() );
            mpObjReader->Update();

            // Create mappers to graphics library
            mpModelMapper = vtkPolyDataMapper::New();
            mpModelMapper->SetInput( mpObjReader->GetOutput() );

            // Actors coordinate geometry, properties, transformation
            mpModelActor = vtkActor::New();
            mpModelActor->SetMapper( mpModelMapper );

            // Turn off lighting on the actors
            //mpModelActor->GetProperty()->SetLighting( false );

            // Add the actors to the scene

            mpRenderer->AddActor( mpModelActor );
            //mpModelActor->SetPosition( 0.02, -0.05, 0.72 );

            const float CRACKER_MODEL_SCALE = 0.975f;
            mpModelActor->SetScale( 0.5*CRACKER_MODEL_SCALE );

            //mpModelActor->SetOrientation( 170.0, 120.0 + 175.0, 0.0 );

            mpModelActor->SetVisibility( this->checkShowModel->isChecked() ? 1 : 0 );
        }

        // VTK can't handle multiple textures on an object, so for now just load in the first
        // texture used by the OBJ file. At the moment we only handle JPG files
        loadTextureForModel( filename );

        // Update the 3D display
        qvtkWidget->update();
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::loadTextureForModel( QString filename )
{
    // Open the OBJ file
    std::ifstream objFile( filename.toStdString() );

    // Find the line containing the name of the MTL file
    std::string mtlFilename;
    bool bFoundMtlFilename = false;
    if ( objFile.is_open() )
    {
        std::string curLine;
        while ( objFile.good() )
        {
            std::getline( objFile, curLine );
            boost::trim( curLine );

            if ( boost::find_first( curLine, "mtllib" ) )
            {
                std::vector<std::string> tokens;
                boost::split( tokens, curLine, boost::is_any_of( "\t " ) );

                if ( tokens.size() >= 2 )
                {
                    mtlFilename = tokens[ 1 ];
                    bFoundMtlFilename = true;
                    break;
                }
            }
        }

        objFile.close();
    }

    if ( !bFoundMtlFilename )
    {
        fprintf( stderr, "Error: Unable to find MTL file in %s\n", filename.toStdString().c_str() );
        return;
    }

    // Open the MTL file
    boost::filesystem::path objFilePath( filename.toStdString() );
    boost::filesystem::path mtlFilePath = objFilePath.parent_path();
    mtlFilePath /= mtlFilename;

    std::ifstream mtlFile( mtlFilePath.string() );

    // Find the first diffuse texture map
    std::string textureFilename;
    bool bFoundTextureFilename = false;
    if ( mtlFile.is_open() )
    {
        std::string curLine;
        while ( mtlFile.good() )
        {
            std::getline( mtlFile, curLine );
            boost::trim( curLine );

            if ( boost::find_first( curLine, "map_Kd" ) )
            {
                std::vector<std::string> tokens;
                boost::split( tokens, curLine, boost::is_any_of( "\t " ) );

                if ( tokens.size() >= 2 )
                {
                    textureFilename = tokens[ 1 ];
                    bFoundTextureFilename = true;
                    break;
                }
            }
        }

        mtlFile.close();
    }

    if ( !bFoundTextureFilename )
    {
        fprintf( stderr, "Error: Unable to find Texture file in %s\n", mtlFilePath.string().c_str() );
        return;
    }

    // Open the texture
    boost::filesystem::path textureFilePath = objFilePath.parent_path();
    textureFilePath /= textureFilename;

    mpModelJpegReader = vtkSmartPointer<vtkJPEGReader>::New();
    mpModelJpegReader->SetFileName( textureFilePath.string().c_str() );
    mpModelJpegReader->Update();

    mpModelTexture = vtkSmartPointer<vtkTexture>::New();
    mpModelTexture->SetInputConnection( mpModelJpegReader->GetOutputPort() );
    mpModelTexture->InterpolateOn();

    // Apply it to the model actor
    mpModelActor->SetTexture( mpModelTexture );
}
