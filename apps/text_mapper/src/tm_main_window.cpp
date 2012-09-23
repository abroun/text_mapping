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
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include "text_mapping/utilities.h"
#include "frame_dialog.h"
#include "tm_main_window.h"
#include "text_detection.h"

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

        // TODO: Fix text map creation
        if ( mFrames.size() == 1 )
        {
            mpFrameTextMap = TextMap::Ptr( new TextMap() );
            mpTextMapSource->SetTextMapPtr( mpFrameTextMap );
        }
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

struct Letter2D
{
    Letter2D( char c, double tl_x, double tl_y, double tr_x, double tr_y,
        double bl_x, double bl_y, double br_x, double br_y )
        : mCharacter( c ),
        mTopLeft( tl_x, tl_y ),
        mTopRight( tr_x, tr_y ),
        mBottomLeft( bl_x, bl_y ),
        mBottomRight( br_x, br_y ) {}

    char mCharacter;
    Eigen::Vector2d mTopLeft;
    Eigen::Vector2d mTopRight;
    Eigen::Vector2d mBottomLeft;
    Eigen::Vector2d mBottomRight;

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

typedef std::vector<Letter2D, Eigen::aligned_allocator<Letter2D> > Letter2DVector;

//--------------------------------------------------------------------------------------------------
void TmMainWindow::onBtnDetectTextClicked()
{
    int32_t curFrameIdx = this->listViewFrames->selectionModel()->currentIndex().row();
    if ( curFrameIdx >= 0 && curFrameIdx < (int32_t)mFrames.size() )
    {
        printf( "Detecting text...\n" );

        LetterList letterList = detectTextInImage( mFrames[ curFrameIdx ].mHighResImage );

        printf( "Found %u letter%s\n", (uint32_t)letterList.size(), ( letterList.size() == 1 ? "" : "s" ) );


        // Add the found letters to the text map
        vtkCamera* pCurCamera = mpRenderer->GetActiveCamera();
        mHighResCamera.setAsActiveCamera( mpRenderer );

        vtkSmartPointer<vtkCellPicker> pPicker = vtkSmartPointer<vtkCellPicker>::New();

        Letter2DVector letters2D;
        letters2D.push_back( Letter2D( 'C', 1091.0, 382.0, 1220.0, 391.0, 1076.0, 535.0, 1180.0, 548.0 ) );

        for ( uint32_t letterIdx = 0; letterIdx < letters2D.size(); letterIdx++ )
        {
            const Letter2D& letter2D = letters2D[ letterIdx ];

            printf( "Trying to pick\n" );

            // Pick each corner in turn
            if ( pPicker->Pick( letter2D.mTopLeft[ 0 ], letter2D.mTopLeft[ 1 ], 0.0, mpRenderer ) )
            {
                printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                double* pickPos = pPicker->GetPickPosition();
            }

            if ( pPicker->Pick( letter2D.mTopRight[ 0 ], letter2D.mTopRight[ 1 ], 0.0, mpRenderer ) )
            {
                printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                double* pickPos = pPicker->GetPickPosition();
            }

            if ( pPicker->Pick( letter2D.mBottomLeft[ 0 ], letter2D.mBottomLeft[ 1 ], 0.0, mpRenderer ) )
            {
                printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                double* pickPos = pPicker->GetPickPosition();
            }

            if ( pPicker->Pick( letter2D.mBottomRight[ 0 ], letter2D.mBottomRight[ 1 ], 0.0, mpRenderer ) )
            {
                printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                double* pickPos = pPicker->GetPickPosition();
            }

            if ( pPicker->Pick( 0.5, 0.5, 0.0, mpRenderer ) )
            {
                printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                double* pickPos = pPicker->GetPickPosition();
            }

        }

        /*Letter letter;
        letter.mMtx = Eigen::Matrix4f::Identity();
        letter.mMtx.block<3,1>( 0, 3 ) = Eigen::Vector3f( 0.0, 0.0, 0.7 );
        letter.mCharacter = 'A';
        letter.mWidth = 0.3;
        letter.mHeight = 0.4;

        mpFrameTextMap->addLetter( letter );
        mpTextMapSource->Modified();*/

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
    mpModelActor->SetVisibility( this->checkShowModel->isChecked() ? 1 : 0 );
    this->qvtkWidget->update();
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

	std::cout << detect_text(image) << endl;

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
    mKinectDepthCamera.setImageSize( 640.0f, 480.0f );
    mKinectColorCamera.setImageSize( 640.0f, 480.0f );
    mHighResCamera.setImageSize( 2272.0f, 1704.0f );

    // TODO: Move away from hard coded cameras
    std::string dataDir = Utilities::getDataDir();
    std::string kinectCalibrationFilename = dataDir + "/point_clouds/calibration_data/windows_kinect.yaml";
    std::string highResCalibrationFilename = dataDir + "/calibration_images/canon_zoom4_cameraMatrix_2.yml";
    std::string highResPoseFilename = dataDir + "/calibration_images/stereo_calibration.yml";

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

    fileStorage.open( highResPoseFilename, cv::FileStorage::READ );
    cv::Mat colorToHighResCameraRotationMatrix;
    cv::Mat colorToHighResCameraTranslationVector;
    fileStorage[ "R" ] >> colorToHighResCameraRotationMatrix;
    fileStorage[ "T" ] >> colorToHighResCameraTranslationVector;
    fileStorage.release();

    // Set up the cameras

    // Depth
    mKinectDepthCamera.setCameraInWorldSpaceMatrix( Eigen::Matrix4d::Identity() );
    mKinectDepthCamera.setCalibrationMatrix(
        Eigen::Map<Eigen::Matrix3d>( (double*)depthCameraCalibrationMatrix.data, 3, 3 ) );

    // Color
    Eigen::Matrix4d kinectColorCameraInWorldSpaceMatrix = Eigen::Matrix4d::Identity();
    kinectColorCameraInWorldSpaceMatrix.block<3,3>( 0, 0 ) = 
        Eigen::Map<Eigen::Matrix3d>( (double*)depthToColorCameraRotationMatrix.data, 3, 3 );
    kinectColorCameraInWorldSpaceMatrix.block<3,1>( 0, 3 ) = 
        Eigen::Map<Eigen::Vector3d>( (double*)depthToColorCameraTranslationVector.data, 3, 1 );

    // HACK: Flipping about the y-axis. This should be done back in the kinect grabber
    kinectColorCameraInWorldSpaceMatrix.block<1,4>( 0, 0 ) = -kinectColorCameraInWorldSpaceMatrix.block<1,4>( 0, 0 );

    mKinectColorCamera.setCameraInWorldSpaceMatrix( kinectColorCameraInWorldSpaceMatrix );
    mKinectColorCamera.setCalibrationMatrix(
        Eigen::Map<Eigen::Matrix3d>( (double*)colorCameraCalibrationMatrix.data, 3, 3 ) );

    // High Resolution
    Eigen::Matrix4d highResCameraInColorCameraSpaceMatrix = Eigen::Matrix4d::Identity();
    highResCameraInColorCameraSpaceMatrix.block<3,3>( 0, 0 ) = 
        Eigen::Map<Eigen::Matrix3d>( (double*)colorToHighResCameraRotationMatrix.data, 3, 3 );
    highResCameraInColorCameraSpaceMatrix.block<3,1>( 0, 3 ) = 
        Eigen::Map<Eigen::Vector3d>( (double*)colorToHighResCameraTranslationVector.data, 3, 1 );
    highResCameraInColorCameraSpaceMatrix( 1, 3 ) = -highResCameraInColorCameraSpaceMatrix( 1, 3 );

    // HACK: Flipping about the x-axis.
    //highResCameraInColorCameraSpaceMatrix.block<1,4>( 1, 0 ) = -highResCameraInColorCameraSpaceMatrix.block<1,4>( 1, 0 );

    Eigen::Matrix4d highResCameraInWorldSpaceMatrix =
        kinectColorCameraInWorldSpaceMatrix*highResCameraInColorCameraSpaceMatrix;

    // HACK: Flipping about the x-axis.
    //highResCameraInWorldSpaceMatrix.block<1,4>( 1, 0 ) = -highResCameraInColorCameraSpaceMatrix.block<1,4>( 1, 0 );

    const Eigen::Vector3d HIGH_RES_CAMERA_OFFSET( -0.04, 0.02, 0.0 );
    const float HIGH_RES_CAMERA_FOV_SCALE = 1.0/0.9;

    mHighResCamera.setCameraInWorldSpaceMatrix( highResCameraInWorldSpaceMatrix );
    mHighResCamera.setCalibrationMatrix(
        Eigen::Map<Eigen::Matrix3d>( (double*)highResCameraCalibrationMatrix.data, 3, 3 )*HIGH_RES_CAMERA_FOV_SCALE );
    mHighResCamera.setClipPlanes( 0.55, 2.0 );

    mHighResCamera.tweakLookAtPos( HIGH_RES_CAMERA_OFFSET );

    mKinectDepthCamera.setClipPlanes( 0.01, 2.0 );
    mKinectColorCamera.setClipPlanes( 0.01, 2.0 );
    mHighResCamera.setClipPlanes( 0.01, 2.0 );

    // Display them in the VTK renderer
    mKinectDepthCamera.showInRenderer( mpRenderer );
    mKinectDepthCamera.setColor( 0, 255, 0 );
    mKinectColorCamera.showInRenderer( mpRenderer );
    mHighResCamera.showInRenderer( mpRenderer );
    mHighResCamera.setColor( 0, 0, 255 );

    mHighResCamera.addPickPoint( Eigen::Vector2d( 1057.0, 124.0 ) );
    mHighResCamera.addPickPoint( Eigen::Vector2d( 1523.0, 176.0 ) );
    mHighResCamera.addPickPoint( Eigen::Vector2d( 1025.0, 1556.0 ) );
    mHighResCamera.addPickPoint( Eigen::Vector2d( 1496.0, 1562.0 ) );
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
