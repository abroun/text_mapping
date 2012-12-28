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
// File: mmb_main_window.cpp
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <vtkCellPicker.h>
#include <vtkProperty.h>
#include <vtkProp3DCollection.h>
#include <vtkRenderWindow.h>
#include <QtGui/qfiledialog.h>
#include <Eigen/Geometry>
#include "text_mapping/utilities.h"
#include "mmb_main_window.h"

//--------------------------------------------------------------------------------------------------
static const float DEFAULT_LETTER_WIDTH = 0.02f;
static const float DEFAULT_LETTER_HEIGHT = 0.03f;

//--------------------------------------------------------------------------------------------------
// MmbMainWindow
//--------------------------------------------------------------------------------------------------
MmbMainWindow::MmbMainWindow()
    : mDoubleClickStartTime( std::clock() ),
      mLastWidth( DEFAULT_LETTER_WIDTH ),
      mLastHeight( DEFAULT_LETTER_HEIGHT )
{
    setupUi( this );
    mpLetterListModel = QSharedPointer<QStringListModel>( new QStringListModel() );
    this->listLetters->setModel( &(*mpLetterListModel) );
    
    // Create a text map, and prepare to display it
    /*mpModelTextMap = TextMap::Ptr( new TextMap() );
    Letter letter;
    letter.mMtx = Eigen::Matrix4f::Identity();
    letter.mMtx.block<3,1>( 0, 3 ) = Eigen::Vector3f( 0.0, 0.0, 0.0 );
    letter.mWidth = 0.2f;
    letter.mHeight = 0.3f;
    letter.mCharacter = 'A';
    mpModelTextMap->addLetter( letter );
    
    letter.mMtx = Eigen::Matrix4f::Identity();
    letter.mMtx.block<3,1>( 0, 3 ) = Eigen::Vector3f( 0.3, 0.1, 0.0 );
    letter.mWidth = 0.2f;
    letter.mHeight = 0.3f;
    letter.mCharacter = '9';
    mpModelTextMap->addLetter( letter );*/
    
    mpTextMapSource = vtkSmartPointer<vtkTextMapSource>::New();
    //mpTextMapSource->SetTextMapPtr( mpModelTextMap );
    
    mpTextMapMapper = vtkPolyDataMapper::New();
    mpTextMapMapper->SetInput( mpTextMapSource->GetOutput() );

    mpTextMapActor = vtkActor::New();
    mpTextMapActor->SetMapper( mpTextMapMapper );
    
    // Turn off lighting on the actors
    //mpTextMapActor->GetProperty()->SetLighting( false );
    
    // Load in a texture containing letters for the text map actor
    mpLettersJpegReader = vtkSmartPointer<vtkJPEGReader>::New();

    std::string fontFilename = Utilities::getDataDir() + "/font/letters.jpg";
    mpLettersJpegReader->SetFileName( fontFilename.c_str() );
    mpLettersJpegReader->Update();

    mpLettersTexture = vtkSmartPointer<vtkTexture>::New();
    mpLettersTexture->SetInputConnection( mpLettersJpegReader->GetOutputPort() );
    mpLettersTexture->InterpolateOn(); 
    
    // Apply it to the text map actor
    mpTextMapActor->SetTexture( mpLettersTexture ); 
    
    // Set up a renderer and connect it to QT
    mpRenderer = vtkRenderer::New();
    qvtkWidget->GetRenderWindow()->AddRenderer( mpRenderer );

    mpRenderer->SetBackground( 0.0, 0.0, 0.0 );
    
    mpRenderer->AddActor( mpTextMapActor );

    onNew();    // Create a default text map
    
    // Setup a callback object to listen for input events from the VTK widget
    mpOnEventCallback = vtkCallbackCommand::New();
    mpOnEventCallback->SetCallback( onInteractorEvent );
    mpOnEventCallback->SetClientData( this );

    qvtkWidget->GetInteractor()->AddObserver( vtkCommand::LeftButtonPressEvent, mpOnEventCallback );
    
    // Hook up signals
    connect( this->action_New, SIGNAL( triggered() ), this, SLOT( onNew() ) );
    connect( this->action_Open, SIGNAL( triggered() ), this, SLOT( onOpen() ) );
    connect( this->action_Save, SIGNAL( triggered() ), this, SLOT( onSave() ) );
    connect( this->action_Save_As, SIGNAL( triggered() ), this, SLOT( onSaveAs() ) );
    connect( this->action_Quit, SIGNAL( triggered() ), this, SLOT( close() ) );
    connect( this->action_Set_Model, SIGNAL( triggered() ), this, SLOT( onSetModel() ) );
    connect( this->listLetters->selectionModel(), 
             SIGNAL( currentChanged( const QModelIndex&, const QModelIndex& ) ), 
             this, 
             SLOT( onCurrentLetterChanged( const QModelIndex&, const QModelIndex& ) ) );
    
    connect( this->btnDeleteLetter, SIGNAL( clicked() ), this, SLOT( onBtnDeleteLetterClicked() ) );
    connect( this->btnLetterLeft, SIGNAL( clicked() ), this, SLOT( onBtnLetterLeftClicked() ) );
    connect( this->btnLetterRight, SIGNAL( clicked() ), this, SLOT( onBtnLetterRightClicked() ) );
    connect( this->btnLetterUp, SIGNAL( clicked() ), this, SLOT( onBtnLetterUpClicked() ) );
    connect( this->btnLetterDown, SIGNAL( clicked() ), this, SLOT( onBtnLetterDownClicked() ) );
    connect( this->lineEditCharacter, SIGNAL( textEdited( const QString& ) ),
             this, SLOT( onCharacterTextEdited( const QString& ) ) );
    connect( this->spinWidth, SIGNAL( valueChanged( double ) ), 
             this, SLOT( onWidthOrHeightValueChanged( double ) ) );
    connect( this->spinHeight, SIGNAL( valueChanged( double ) ), 
             this, SLOT( onWidthOrHeightValueChanged( double ) ) );
    connect( this->checkHideTextMap, SIGNAL( clicked() ),
            this, SLOT( onCheckHideTextMapClicked() ) );
}

//--------------------------------------------------------------------------------------------------
MmbMainWindow::~MmbMainWindow()
{
}


//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onNew()
{
    mTextMapFilename = "";
    mpModelTextMap = TextMap::Ptr( new TextMap() );
    mpTextMapSource->SetTextMapPtr( mpModelTextMap );
    
    if ( NULL != mpObjReader )
    {
        mpObjReader->SetFileName( "" );
        mpObjReader->Update();
    }
    
    refreshLetterList();
    mpTextMapActor->SetVisibility( !this->checkHideTextMap->isChecked() );
    
    // Update the 3D display
    qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onOpen()
{
    std::string textMapsDir = Utilities::getDataDir() + "/text_maps";

    QString filename = QFileDialog::getOpenFileName( this,
         tr( "Open Text Map" ), textMapsDir.c_str(), tr("Text Map Files (*.map)") );

    if ( !filename.isEmpty() )
    {
        std::string newTextMapFilename = filename.toStdString();
        TextMap::Ptr pNewTextMap = TextMap::loadTextMapFromFile( newTextMapFilename );
        if ( NULL == pNewTextMap )
        {
            fprintf( stderr, "Error: Unable to open %s\n", newTextMapFilename.c_str() );
        }
        else
        {
            mpModelTextMap = pNewTextMap;
            mpTextMapSource->SetTextMapPtr( mpModelTextMap );
            mTextMapFilename = newTextMapFilename;
            
            mpTextMapActor->SetVisibility( !this->checkHideTextMap->isChecked() );

            const std::string& modelFilename = mpModelTextMap->getModelFilename();
            if ( modelFilename == TextMap::NO_MODEL_FILENAME )
            {
                if ( NULL != mpObjReader )
                {
                    mpObjReader->SetFileName( "" );
                    mpObjReader->Update();
                }
                
                // Update the 3D display
                qvtkWidget->update();
            }
            else
            {
                loadObjModel( modelFilename.c_str() );
            }
            
            // Display the letters in a list
            refreshLetterList();
        }
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onSave()
{
    if ( NULL != mpModelTextMap )
    {
        if ( mTextMapFilename == "" )
        {
            onSaveAs();
        }
        else
        {
            mpModelTextMap->saveToFile( mTextMapFilename );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onSaveAs()
{
    if ( NULL != mpModelTextMap )
    {
        std::string textMapsDir = Utilities::getDataDir() + "/text_maps";
        QString filename = QFileDialog::getSaveFileName( this,
            tr( "Save Text Map" ), textMapsDir.c_str(), tr("Text Map Files (*.map)") );
        
        if ( !filename.isEmpty() )
        {
            mTextMapFilename = filename.toStdString();
            mpModelTextMap->saveToFile( mTextMapFilename );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onSetModel()
{
    std::string dataDir = Utilities::getDataDir();
    QString filename = QFileDialog::getOpenFileName( this,
         tr( "Open Object Model" ), dataDir.c_str(), tr("Object Files (*.obj)") );

    mpTextMapActor->SetVisibility( !this->checkHideTextMap->isChecked() );
    loadObjModel( filename );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onCurrentLetterChanged( const QModelIndex& current, const QModelIndex& previous )
{
    mCurLetterIdx = current.row();
    if ( mCurLetterIdx < mpModelTextMap->getNumLetters() )
    {
        mbSelectingNewLetter = true;
        
        const Letter& letter = mpModelTextMap->getLetter( mCurLetterIdx );
        
        char letterStringBuffer[] = { letter.mCharacter, '\0' };
        this->lineEditCharacter->setText( letterStringBuffer );
        this->spinWidth->setValue( letter.mWidth );
        this->spinHeight->setValue( letter.mHeight );
        this->groupLetterEdit->setEnabled( true );
        
        mbSelectingNewLetter = false;
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onBtnDeleteLetterClicked()
{
    if ( mCurLetterIdx < mpModelTextMap->getNumLetters() )
    {
        mpModelTextMap->deleteLetter( mCurLetterIdx );
        
        mpTextMapSource->Modified();
        qvtkWidget->update();
        refreshLetterList();
        
        this->groupLetterEdit->setEnabled( false ); 
        
        // Try to find the index of a new letter to select
        if ( mCurLetterIdx >= mpModelTextMap->getNumLetters() )
        {
            if ( mpModelTextMap->getNumLetters() > 0 )
            {
                mCurLetterIdx = mpModelTextMap->getNumLetters() - 1;
            }
        }
        
        if ( mCurLetterIdx < mpModelTextMap->getNumLetters() )
        {
            QModelIndex selectionIdx = mpLetterListModel->index( mCurLetterIdx, 0 );
            
            this->listLetters->selectionModel()->select( 
                selectionIdx, QItemSelectionModel::SelectCurrent );
        
            // TODO: Find away of having the selection model make this call...
            onCurrentLetterChanged( selectionIdx, selectionIdx );
        }
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onBtnLetterLeftClicked()
{
    shiftLetter( -1, 0 );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onBtnLetterRightClicked()
{
    shiftLetter( 1, 0 );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onBtnLetterUpClicked()
{
    shiftLetter( 0, -1 );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onBtnLetterDownClicked()
{
    shiftLetter( 0, 1 );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onCharacterTextEdited( const QString& characterText )
{
    // This will only be called when the text is edited by a human (i.e. not programmatically)
    if ( mCurLetterIdx < mpModelTextMap->getNumLetters() )
    {
        char character = ' ';
        if ( characterText.length() >= 1 )
        {
            character = characterText[ 0 ].toAscii();
        }
        
        Letter& letter = mpModelTextMap->getLetter( mCurLetterIdx );
        letter.mCharacter = character;
        
        mpTextMapSource->Modified();
        qvtkWidget->update();
        refreshLetterList();
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onWidthOrHeightValueChanged( double value )
{
    if ( !mbSelectingNewLetter
        && mCurLetterIdx < mpModelTextMap->getNumLetters() )
    {
         Letter& letter = mpModelTextMap->getLetter( mCurLetterIdx );
         letter.mWidth = this->spinWidth->value();
         letter.mHeight = this->spinHeight->value();
         
         mLastWidth = letter.mWidth;
         mLastHeight = letter.mHeight;

         mpTextMapSource->Modified();
         qvtkWidget->update();
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onCheckHideTextMapClicked( bool bChecked )
{
    mpTextMapActor->SetVisibility( !this->checkHideTextMap->isChecked() );
    qvtkWidget->update();
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onInteractorEvent( vtkObject* pCaller, unsigned long eid, 
                                       void* pClientdata, void* pCalldata )
{
#ifdef WIN32
    const float MAX_TIME_FOR_DOUBLE_CLICK = 0.2f;  // Time in seconds
#else
    const float MAX_TIME_FOR_DOUBLE_CLICK = 0.01f;  // Time in seconds
#endif
    
    MmbMainWindow* pWin = (MmbMainWindow*)pClientdata;
    vtkRenderWindowInteractor* pInteractor = pWin->qvtkWidget->GetInteractor();

    switch ( eid )
    {
        case vtkCommand::LeftButtonPressEvent:
        {
            std::clock_t clickTime = std::clock();

            if ( ((float)(clickTime - pWin->mDoubleClickStartTime))/CLOCKS_PER_SEC <= MAX_TIME_FOR_DOUBLE_CLICK )
            {
                // Double click detected
                pWin->mDoubleClickStartTime = 0;

                // Pick from this location.
                int* mousePos = pInteractor->GetEventPosition();
                vtkSmartPointer<vtkCellPicker> pPicker = vtkSmartPointer<vtkCellPicker>::New();
                int pickResult = pPicker->Pick( mousePos[ 0 ], mousePos[ 1 ], 0, pWin->mpRenderer );

                if ( 0 != pickResult
                    && NULL != pWin->mpModelTextMap )
                {
                	printf( "Intersected with %i props\n", pPicker->GetProp3Ds()->GetNumberOfItems () );

                    double* pickPos = pPicker->GetPickPosition();
                    double* pickNormal = pPicker->GetMapperNormal();

                    Eigen::Vector3f letterPos( pickPos[ 0 ], pickPos[ 1 ], pickPos[ 2 ] );
                    Eigen::Vector3f letterNormal( pickNormal[ 0 ], pickNormal[ 1 ], pickNormal[ 2 ] );
                    
                    // Pick an arbitrary y-axis for the letter
                    Eigen::Vector3f axisY;
                    if ( fabsf( letterNormal[ 1 ] ) > 0.95f )
                    {
                        // Letter normal is too close to ( 0.0, 1.0, 0.0 )
                        axisY = Eigen::Vector3f( 1.0, 0.0, 0.0 );
                    }
                    else
                    {
                        axisY = Eigen::Vector3f( 0.0, 1.0, 0.0 );
                    }
                    
                    // Now build an orthonormal rotation matrix for the letter
                    Eigen::Vector3f axisX = axisY.cross( letterNormal );
                    axisX.normalize();
                    axisY = letterNormal.cross( axisX );
                    axisY.normalize();
                    
                    Letter letter;
                    letter.mMtx = Eigen::Matrix4f::Identity();
                    letter.mMtx.block<3,1>( 0, 0 ) = axisX;
                    letter.mMtx.block<3,1>( 0, 1 ) = axisY;
                    letter.mMtx.block<3,1>( 0, 2 ) = letterNormal;
                    letter.mMtx.block<3,1>( 0, 3 ) = letterPos;
                    letter.mWidth = pWin->mLastWidth;
                    letter.mHeight = pWin->mLastHeight;
                    letter.mCharacter = 'A';
                    pWin->mpModelTextMap->addLetter( letter );
                    
                    pWin->mpTextMapSource->Modified();
                    pWin->qvtkWidget->update();
                    pWin->refreshLetterList();
                }
            }
            else
            {
                // No double click yet
                pWin->mDoubleClickStartTime = clickTime;
            }

            break;
        }
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::shiftLetter( int32_t horizontalSteps, int32_t verticalSteps )
{
    if ( mCurLetterIdx < mpModelTextMap->getNumLetters() )
    {
         Letter& letter = mpModelTextMap->getLetter( mCurLetterIdx );

         float hShift = 0.01*letter.mWidth;
         float vShift = 0.01*letter.mHeight;

         letter.mMtx.block<3,1>( 0, 3 ) += hShift*horizontalSteps*letter.mMtx.block<3,1>( 0, 0 );
         letter.mMtx.block<3,1>( 0, 3 ) -= vShift*verticalSteps*letter.mMtx.block<3,1>( 0, 1 );
         mpTextMapSource->Modified();
         qvtkWidget->update();
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::loadObjModel( QString filename )
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
        }
        
        // VTK can't handle multiple textures on an object, so for now just load in the first
        // texture used by the OBJ file. At the moment we only handle JPG files
        loadTextureForModel( filename );

        // Update the 3D display
        qvtkWidget->update();
        
        mpModelTextMap->setModelFilename( filename.toStdString() );
    }
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::loadTextureForModel( QString filename )
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

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::refreshLetterList()
{
    // Create a list of strings representing the characters
    QStringList list;
    if ( NULL != mpModelTextMap && mpModelTextMap->getNumLetters() > 0 )
    {
        char shortStringBuffer[ 2 ];
        shortStringBuffer[ 1 ] = '\0';
        
        for ( uint32_t letterIdx = 0; letterIdx < mpModelTextMap->getNumLetters(); letterIdx++ )
        {
            shortStringBuffer[ 0 ] = mpModelTextMap->getLetter( letterIdx ).mCharacter;
            list << shortStringBuffer;
        }
    }
    else
    {
        // Disable the controls for editing the letter
        this->groupLetterEdit->setEnabled( false );
    }
            
    mpLetterListModel->setStringList( list );
}
