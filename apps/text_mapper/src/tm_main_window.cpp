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
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include "text_mapping/utilities.h"
#include "frame_dialog.h"
#include "tm_main_window.h"
#include <opencv2/core/eigen.hpp>

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
    mpKeyPointInstancesActor->GetProperty()->SetPointSize( 3 );

    mpRenderer->AddActor( mpKeyPointInstancesActor );

    // Load in object model
    std::string dataDir = Utilities::getDataDir();
    QString modelFilename = QString( dataDir.c_str() ) + "/models/carrs_crackers.obj";
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

    connect( this->btnLeft, SIGNAL( clicked() ), this, SLOT( onBtnLeftClicked() ) );
    connect( this->btnRight, SIGNAL( clicked() ), this, SLOT( onBtnRightClicked() ) );
    connect( this->btnUp, SIGNAL( clicked() ), this, SLOT( onBtnUpClicked() ) );
    connect( this->btnDown, SIGNAL( clicked() ), this, SLOT( onBtnDownClicked() ) );

    connect( this->checkShowModel, SIGNAL( clicked() ), this, SLOT( onCheckShowModelClicked() ) );

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
        int32_t currentFrameIdx = current.row();
        if ( currentFrameIdx >= 0 && currentFrameIdx < (int32_t)mFrames.size() )
        {
            mFrames[ currentFrameIdx ].tryToLoadImages( false );
            refreshImageDisplays( &mFrames[ currentFrameIdx ] );

            // Create a new text map for the frame
            mpFrameTextMap = TextMap::Ptr( new TextMap() );
            mpTextMapSource->SetTextMapPtr( mpFrameTextMap );
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

        Letter2DVector letters2D = detect_text( mFrames[ curFrameIdx ].mHighResImage );

        printf( "Found %u letter%s\n", (uint32_t)letters2D.size(), ( letters2D.size() == 1 ? "" : "s" ) );


        // Add the found letters to the text map
        //vtkCamera* pCurCamera = mpRenderer->GetActiveCamera();
        //mHighResCamera.setAsActiveCamera( mpRenderer );

        vtkSmartPointer<vtkCellPicker> pPicker = vtkSmartPointer<vtkCellPicker>::New();


        for ( uint32_t letterIdx = 0; letterIdx < letters2D.size(); letterIdx++ )
        {
            printf( "Adding letter %lu of %lu\n", letterIdx + 1, letters2D.size() );

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

			mpFrameTextMap->addLetter( letter );
			mpTextMapSource->Modified();

        }

        

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
		mHighResCamera.addPickPoint( Eigen::Vector2d( 1057.0, 124.0 ) );
		mHighResCamera.addPickPoint( Eigen::Vector2d( 1523.0, 176.0 ) );
		mHighResCamera.addPickPoint( Eigen::Vector2d( 1025.0, 1556.0 ) );
		mHighResCamera.addPickPoint( Eigen::Vector2d( 1496.0, 1562.0 ) );
		mpModelActor->SetVisibility( 1 );
	}
	else
	{
		mHighResCamera.clearPickPoints();
		mpModelActor->SetVisibility( 0 );
	}
    this->qvtkWidget->update();
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
void TmMainWindow::addKeyPointInstanceAtImagePos( const ImageViewDialog* pImageViewDialog, const QPointF& pickPoint )
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

    
    float distanceToSurface = mpPointCloudSource->GetPointCloudPtr()->pickSurface(
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
void TmMainWindow::refreshImageDisplays( const FrameData* pFrameData )
{
    if ( NULL == pFrameData )
    {
        // Hide image view dialogs
        for ( uint32_t dialogIdx = 0; dialogIdx < mpImageViewDialogs.size(); dialogIdx++ )
        {
            mpImageViewDialogs[ dialogIdx ]->hide();
        }

        // Clear the frame display
        mpPointCloudActor->SetVisibility( 0 );
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

        mpPointCloudSource->SetPointCloudPtr( pFrameData->mpKinectDepthPointCloud );
        mpPointCloudActor->SetVisibility( 1 );
    }

    refreshKeyPointInstances();
    this->qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::refreshKeyPointInstances()
{
    // Create a list of the key point instances to display
    std::vector<vtkKeyPointInstancesSource::InstanceData> keyPointInstances;

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
                if ( (int32_t)keyPointIdx == selectedKeyPointIdx )
                {
                    // Yellow for selected frame instance
                    keyPointInstances.push_back( vtkKeyPointInstancesSource::InstanceData(
                        *pInstance, 255, 255, 0 ) );
                }
                else
                {
                    // Green for normal frame instance
                    keyPointInstances.push_back( vtkKeyPointInstancesSource::InstanceData(
                        *pInstance, 0, 255, 0 ) );
                }
            }
        }
    }

    // Add in model instances if needed
    if ( this->checkShowModel->isChecked() )
    {
        for ( uint32_t keyPointIdx = 0; keyPointIdx < mKeyPoints.size(); keyPointIdx++ )
        {
            const KeyPointInstance* pInstance = mKeyPoints[ keyPointIdx ].getKeyPointModelInstance();
            if ( NULL != pInstance )
            {
                if ( (int32_t)keyPointIdx == selectedKeyPointIdx )
                {
                    // Yellow for selected model instance
                    keyPointInstances.push_back( vtkKeyPointInstancesSource::InstanceData(
                        *pInstance, 255, 255, 0 ) );
                }
                else
                {
                    // Green for normal model instance
                    keyPointInstances.push_back( vtkKeyPointInstancesSource::InstanceData(
                        *pInstance, 0, 255, 0 ) );
                }
            }
        }
    }

    // Display the key point instances
    mpKeyPointInstancesSource->SetKeyPointInstances( keyPointInstances );
    this->qvtkWidget->update();
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
            mpModelActor->SetPosition( 0.02, -0.05, 0.72 );
            mpModelActor->SetScale( 0.5 );
            mpModelActor->SetOrientation( 170.0, 120.0 + 175.0, 0.0 );

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
