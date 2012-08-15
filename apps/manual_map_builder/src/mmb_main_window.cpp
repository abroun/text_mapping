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
#include <string>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <QtGui/qfiledialog.h>
#include <Eigen/Geometry>
#include "mmb_main_window.h"

//--------------------------------------------------------------------------------------------------
// MmbMainWindow
//--------------------------------------------------------------------------------------------------
MmbMainWindow::MmbMainWindow()
{
    setupUi( this );
    
    // Create a text map, and prepare to display it
    mpModelTextMap = TextMap::Ptr( new TextMap() );
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
    mpModelTextMap->addLetter( letter );
    
    mpTextMapSource = vtkSmartPointer<vtkTextMapSource>::New();
    mpTextMapSource->SetTextMapPtr( mpModelTextMap );
    
    mpTextMapMapper = vtkPolyDataMapper::New();
    mpTextMapMapper->SetInput( mpTextMapSource->GetOutput() );

    mpTextMapActor = vtkActor::New();
    mpTextMapActor->SetMapper( mpTextMapMapper );
    
    // Turn off lighting on the actors
    //mpTextMapActor->GetProperty()->SetLighting( false );
    
    // Load in a texture containing letters for the text map actor
    mpLettersJpegReader = vtkSmartPointer<vtkJPEGReader>::New();
    mpLettersJpegReader->SetFileName( "../data/font/letters.jpg" );
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

    // Hook up signals
    connect( action_Open, SIGNAL( triggered() ), this, SLOT( onOpen() ) );
    connect( action_Save, SIGNAL( triggered() ), this, SLOT( onSave() ) );
    connect( action_Quit, SIGNAL( triggered() ), this, SLOT( close() ) );
}

//--------------------------------------------------------------------------------------------------
MmbMainWindow::~MmbMainWindow()
{
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onOpen()
{
    QString filename = QFileDialog::getOpenFileName( this,
         tr( "Open Object Model" ), "../data", tr("Object Files (*.obj)") );

    loadObjModel( filename );
}

//--------------------------------------------------------------------------------------------------
void MmbMainWindow::onSave()
{
    
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
        printf( "Opened obj file\n" );
        
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