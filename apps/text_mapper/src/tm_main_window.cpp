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
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include "text_mapping/utilities.h"
#include "frame_dialog.h"
#include "tm_main_window.h"

//--------------------------------------------------------------------------------------------------
// TmMainWindow
//--------------------------------------------------------------------------------------------------
TmMainWindow::TmMainWindow()
{
    setupUi( this );

    mpFrameListModel = QSharedPointer<QStringListModel>( new QStringListModel() );
    this->listViewFrames->setModel( &(*mpFrameListModel) );

    // Set up a renderer and connect it to QT
    mpRenderer = vtkRenderer::New();
    qvtkWidget->GetRenderWindow()->AddRenderer( mpRenderer );

    mpRenderer->SetBackground( 0.0, 0.0, 0.0 );

    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();
      points->InsertNextPoint(0,0,0);
      points->InsertNextPoint(1,1,1);
      points->InsertNextPoint(2,2,2);
      vtkSmartPointer<vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
      polydata->SetPoints(points);

    // Set up the pipeline to render a point cloud
    mpPointCloudSource = vtkSmartPointer<vtkPointCloudSource>::New();
    mpCubeSource = vtkSmartPointer<vtkCubeSource>::New();
    mpPointCloudGlyphs = vtkSmartPointer<vtkGlyph3D>::New();
    mpPointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpPointCloudActor = vtkSmartPointer<vtkActor>::New();

    mpCubeSource->SetXLength( 0.001 );
    mpCubeSource->SetYLength( 0.001 );
    mpCubeSource->SetZLength( 0.001 );
    mpPointCloudGlyphs->SetSourceConnection( mpCubeSource->GetOutputPort() );
    //mpPointCloudGlyphs->SetInput( mpPointCloudSource->GetOutput() );
    //mpPointCloudGlyphs->SetInput( polydata );

    //mpPointCloudMapper->SetInputConnection( mpPointCloudGlyphs->GetOutputPort() );
    mpPointCloudMapper->SetInputConnection( mpPointCloudSource->GetOutputPort() );
    //mpPointCloudMapper->SetInput( polydata );
    //mpPointCloudMapper->SetColorModeToDefault();
	//	mpPointCloudMapper->SetScalarRange(-10.0, 10.0);
	//	mpPointCloudMapper->SetScalarVisibility(1);


    //mpPointCloudMapper->SetInputConnection( mpCubeSource->GetOutputPort() );

    mpPointCloudActor->SetMapper( mpPointCloudMapper );

    //mpPointCloudActor->GetProperty()->SetRepresentationToPoints();

    mpRenderer->AddActor( mpPointCloudActor );

    std::string dataDir = Utilities::getDataDir();
    QString modelFilename = QString( dataDir.c_str() ) + "/models/carrs_crackers.obj";
    loadObjModel( modelFilename );

    loadCameras();

    // Hook up signals
    connect( this->listViewFrames->selectionModel(), 
             SIGNAL( currentChanged( const QModelIndex&, const QModelIndex& ) ), 
             this, 
             SLOT( onCurrentFrameChanged( const QModelIndex&, const QModelIndex& ) ) );
    
    connect( this->btnAddFrame, SIGNAL( clicked() ), this, SLOT( onBtnAddFrameClicked() ) );
    connect( this->btnEditFrame, SIGNAL( clicked() ), this, SLOT( onBtnEditFrameClicked() ) );
    connect( this->btnDeleteFrame, SIGNAL( clicked() ), this, SLOT( onBtnDeleteFrameClicked() ) );
    connect( this->btnDetectText, SIGNAL( clicked() ), this, SLOT( onBtnDetectTextClicked() ) );
}

//--------------------------------------------------------------------------------------------------
TmMainWindow::~TmMainWindow()
{
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onCurrentFrameChanged( const QModelIndex& current, const QModelIndex& previous )
{
    // TODO: Do stuff...
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnAddFrameClicked()
{
    FrameData newFrameData;
    if ( FrameDialog::createNewFrame( &newFrameData ) )
    {
        refreshImageDisplays( newFrameData );

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

        refreshImageDisplays( mFrames[ curFrameIdx ] );
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
    }
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnDetectTextClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        printf( "Detecting text...\n" );

        LetterList letterList = detectTextInImage( mFrames[ curFrameIdx ].mHighResImage );

        printf( "Found %u letter%s\n", (uint32_t)letterList.size(), ( letterList.size() == 1 ? "" : "s" ) );
    }
    else
    {
        printf( "Error: No frame selected\n" );
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
void TmMainWindow::refreshImageDisplays( const FrameData& frameData )
{
    mHighResImageViewDialog.setImage( frameData.mHighResImage );
    mHighResImageViewDialog.setWindowTitle( "High Res Image" );
    mHighResImageViewDialog.show();
    mKinectColorImageViewDialog.setImage( frameData.mKinectColorImage );
    mKinectColorImageViewDialog.setWindowTitle( "Kinect Color Image" );
    mKinectColorImageViewDialog.show();
    mKinectDepthColorImageViewDialog.setImage( frameData.mpKinectDepthPointCloud->getImage() );
    mKinectDepthColorImageViewDialog.setWindowTitle( "Kinect Depth Camera Color Image" );
    mKinectDepthColorImageViewDialog.show();

    mpPointCloudSource->SetPointCloudPtr( frameData.mpKinectDepthPointCloud );
    this->qvtkWidget->update();

    int32_t idx0 = frameData.mpKinectDepthPointCloud->getPointIdxAtImagePos( 324, 200 );
    int32_t idx1 = frameData.mpKinectDepthPointCloud->getPointIdxAtImagePos( 372, 198 );
    if ( PointCloud::INVALID_POINT_IDX != idx0 && PointCloud::INVALID_POINT_IDX != idx1 )
    {
    	Eigen::Vector3f p0 = frameData.mpKinectDepthPointCloud->getPointWorldPos( idx0 );
    	Eigen::Vector3f p1 = frameData.mpKinectDepthPointCloud->getPointWorldPos( idx1 );

    	printf( "Distance between points is %f\n", (p1 - p0).norm() );
    }
}

//--------------------------------------------------------------------------------------------------
TmMainWindow::LetterList TmMainWindow::detectTextInImage( cv::Mat image )
{
    // TODO: Magic text detection stuff

    // TODO: Create a list of letters to send back
    LetterList letterList;

    // AB: An example of how the existing letter class could be used...
    float letterX = 100.0f;
    float letterY = 300.0f;

    Letter letter;
    letter.mMtx = Eigen::Matrix4f::Identity();
    letter.mMtx.block<2,1>( 0, 3 ) = Eigen::Vector2f( letterX, letterY );
    letter.mCharacter = 'A';
    letterList.push_back( letter );


    return letterList;
}

//--------------------------------------------------------------------------------------------------
void TmMainWindow::loadCameras()
{
    float HALF_KINECT_IMAGE_HEIGHT = 640.0f/2.0f;

    // TODO: Move away from hard coded cameras
    std::string dataDir = Utilities::getDataDir();
    std::string kinectCalibrationFilename = dataDir + "/point_clouds/calibration_data/windows_kinect.yaml";

    // Load in the Kinect calibration file
    cv::FileStorage fs;
    fs.open( kinectCalibrationFilename, cv::FileStorage::READ );

    // Read out matrices for the color and depth camera on the Kinect
    cv::Mat depthCameraCalibrationMatrix;
    cv::Mat colorCameraCalibrationMatrix;

    fs[ "DepthCameraCalibrationMtx" ] >> depthCameraCalibrationMatrix;
    fs[ "ColorCameraCalibrationMtx" ] >> colorCameraCalibrationMatrix;

    cv::Mat depthToColorCameraRotationMatrix;
    cv::Mat depthToColorCameraTranslationVector;

    fs[ "DepthToColorCameraRotation" ] >> depthToColorCameraRotationMatrix;
    fs[ "DepthToColorCameraTranslation" ] >> depthToColorCameraTranslationVector;

    // Set up the cameras
    mKinectDepthCamera.setCameraInWorldSpaceMatrix( Eigen::Matrix4f::Identity() );
    mKinectDepthCamera.setCalibrationMatrix(
        Eigen::Map<Eigen::Matrix3d>( (double*)colorCameraCalibrationMatrix.data, 3, 3 ),
        HALF_KINECT_IMAGE_HEIGHT );

    // Display them in the VTK renderer
    mKinectDepthCamera.showInRenderer( mpRenderer );
    //mKinectColorCamera.showInRenderer( mpRenderer );
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
            mpModelActor->SetPosition( 0.02, -0.04, 0.72 );
            mpModelActor->SetScale( 0.5 );
            mpModelActor->SetOrientation( 170.0, 120.0, 0.0 );
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
