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
#include <Eigen/Dense>
#include <vtkCellArray.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include "text_mapping/utilities.h"
#include "camera.h"

//--------------------------------------------------------------------------------------------------
// Camera
//--------------------------------------------------------------------------------------------------
Camera::Camera()
{
    mpVtkCamera = vtkSmartPointer<vtkCamera>::New();
    mpVtkCameraActor = vtkSmartPointer<vtkCameraActor>::New();
    mpVtkCameraActor->SetCamera( mpVtkCamera );
    setImageSize( 640.0, 480.0 );

    mpPickLinesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mpPickLinesActor = vtkSmartPointer<vtkActor>::New();
    mpPickLinesActor->SetMapper( mpPickLinesMapper );

    mTotalOffset = Eigen::Vector3d::Zero();
}

//--------------------------------------------------------------------------------------------------
Camera::~Camera()
{
}

//--------------------------------------------------------------------------------------------------
void Camera::setImageSize( float imageWidth, float imageHeight )
{
    mImageWidth = imageWidth;
    mImageHeight = imageHeight;
    mpVtkCameraActor->SetWidthByHeightRatio( imageWidth/imageHeight );
}

//--------------------------------------------------------------------------------------------------
void Camera::setCameraInWorldSpaceMatrix( const Eigen::Matrix4d& cameraInWorldSpaceMatrix )
{
    const Eigen::Vector3d& pos = cameraInWorldSpaceMatrix.block<3,1>( 0, 3 );
    const Eigen::Vector3d& axisZ = cameraInWorldSpaceMatrix.block<3,1>( 0, 2 );
    const Eigen::Vector3d& axisY = cameraInWorldSpaceMatrix.block<3,1>( 0, 1 );
    Eigen::Vector3d focalPoint = pos + axisZ;

    mpVtkCamera->SetPosition( pos[ 0 ], pos[ 1 ], pos[ 2 ] );
    mpVtkCamera->SetFocalPoint( focalPoint[ 0 ], focalPoint[ 1 ], focalPoint[ 2 ] );
    mpVtkCamera->SetViewUp( -axisY[ 0 ], -axisY[ 1 ], -axisY[ 2 ] );
}

//--------------------------------------------------------------------------------------------------
void Camera::setCalibrationMatrix( const Eigen::Matrix3d& calibrationMtx )
{
    float halfImageHeight = mImageHeight / 2.0f;

    float halfViewAngle = Utilities::radToDeg(
        atan2f( halfImageHeight, calibrationMtx( 1, 1 ) ) );

    printf( "f_y %f and half height %f\n", calibrationMtx( 1, 1 ), halfImageHeight );
    printf( "Half view angle is %f degrees\n", halfViewAngle );

    mpVtkCamera->SetViewAngle( 2.0*halfViewAngle );
    mpVtkCamera->SetClippingRange( calibrationMtx( 1, 1 )/1000.0, 2.0 );
}

//--------------------------------------------------------------------------------------------------
void Camera::setClipPlanes( float near, float far )
{
    mpVtkCamera->SetClippingRange(  near, far );
}

//--------------------------------------------------------------------------------------------------
void Camera::addPickPoint( const Eigen::Vector2d& screenPos )
{
    Eigen::Vector2d normalisedScreenPos( 
        (2.0*screenPos[ 0 ])/mImageWidth - 1.0, (2.0*screenPos[ 1 ])/mImageHeight - 1.0 );

    mPickPoints.push_back( normalisedScreenPos );

    updatePickLines();
}

//--------------------------------------------------------------------------------------------------
void Camera::clearPickPoints()
{
	mPickPoints.clear();
	updatePickLines();
}

//--------------------------------------------------------------------------------------------------
void Camera::getLineForPickPoint( const Eigen::Vector2d& screenPos, Eigen::Vector3d* pLineStartOut, Eigen::Vector3d* pLineDirOut )
{
	Eigen::Vector2d normalisedScreenPos( 
        (2.0*screenPos[ 0 ])/mImageWidth - 1.0, (2.0*screenPos[ 1 ])/mImageHeight - 1.0 );

    double halfVerticalAngle = 0.5*Utilities::degToRad( mpVtkCamera->GetViewAngle() );
    double verticalLength = tan( halfVerticalAngle );
    double horizontalLength = verticalLength*(mImageWidth/mImageHeight);

    Eigen::Vector3d cameraPos;
    Eigen::Vector3d cameraAxisX;
    Eigen::Vector3d cameraAxisY;
    Eigen::Vector3d cameraAxisZ;
    mpVtkCamera->GetPosition( cameraPos[ 0 ], cameraPos[ 1 ], cameraPos[ 2 ] );
    mpVtkCamera->GetDirectionOfProjection( cameraAxisZ[ 0 ], cameraAxisZ[ 1 ], cameraAxisZ[ 2 ] );
    mpVtkCamera->GetViewUp( cameraAxisY[ 0 ], cameraAxisY[ 1 ], cameraAxisY[ 2 ] );

    cameraAxisX = cameraAxisY.cross( cameraAxisZ );

    Eigen::Vector3d startPos = cameraPos;
    Eigen::Vector3d rayDir = cameraAxisZ 
        - normalisedScreenPos[ 0 ]*horizontalLength*cameraAxisX
        - normalisedScreenPos[ 1 ]*verticalLength*cameraAxisY;

    rayDir.normalize();

    *pLineStartOut = startPos;
    *pLineDirOut = rayDir;
}

//--------------------------------------------------------------------------------------------------
void Camera::updatePickLines()
{
    // Now draw lines from the pick points
    double halfVerticalAngle = 0.5*Utilities::degToRad( mpVtkCamera->GetViewAngle() );
    double verticalLength = tan( halfVerticalAngle );
    double horizontalLength = verticalLength*(mImageWidth/mImageHeight);

    Eigen::Vector3d cameraPos;
    Eigen::Vector3d cameraAxisX;
    Eigen::Vector3d cameraAxisY;
    Eigen::Vector3d cameraAxisZ;
    mpVtkCamera->GetPosition( cameraPos[ 0 ], cameraPos[ 1 ], cameraPos[ 2 ] );
    mpVtkCamera->GetDirectionOfProjection( cameraAxisZ[ 0 ], cameraAxisZ[ 1 ], cameraAxisZ[ 2 ] );
    mpVtkCamera->GetViewUp( cameraAxisY[ 0 ], cameraAxisY[ 1 ], cameraAxisY[ 2 ] );

    cameraAxisX = cameraAxisY.cross( cameraAxisZ );

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    for ( uint32_t pickPointIdx = 0; pickPointIdx < mPickPoints.size(); pickPointIdx++ )
    {
        const Eigen::Vector2d& p = mPickPoints[ pickPointIdx ];

        Eigen::Vector3d startPos = cameraPos;
        Eigen::Vector3d rayDir = cameraAxisZ 
            - p[ 0 ]*horizontalLength*cameraAxisX
            - p[ 1 ]*verticalLength*cameraAxisY;

        rayDir.normalize();

        Eigen::Vector3d endPos = startPos + 2.0*rayDir;

        // Create the line
        points->InsertNextPoint( startPos.data() );
        points->InsertNextPoint( endPos.data() );

        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId( 0, 2*pickPointIdx );
        line->GetPointIds()->SetId( 1, 2*pickPointIdx + 1 );

        // Store the line
        lines->InsertNextCell( line );
    }

    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
 
    // Add the points and lines to the dataset
    linesPolyData->SetPoints( points );
    linesPolyData->SetLines( lines );

    mpPickLinesMapper->SetInput( linesPolyData );
}

//--------------------------------------------------------------------------------------------------
void Camera::tweakLookAtPos( const Eigen::Vector3d& offset )
{
    Eigen::Vector3d focalPoint;
    mpVtkCamera->GetFocalPoint( focalPoint[ 0 ], focalPoint[ 1 ], focalPoint[ 2 ] );

    mTotalOffset += offset;
    std::cout << "Total offset is now " << mTotalOffset << "\n";

    focalPoint += offset;
    mpVtkCamera->SetFocalPoint( focalPoint[ 0 ], focalPoint[ 1 ], focalPoint[ 2 ] );

    updatePickLines();
}

//--------------------------------------------------------------------------------------------------
void Camera::showInRenderer( vtkRenderer* pRenderer )
{
    pRenderer->AddActor( mpVtkCameraActor );
    pRenderer->AddActor( mpPickLinesActor );
}

//--------------------------------------------------------------------------------------------------
void Camera::setAsActiveCamera( vtkRenderer* pRenderer )
{
    pRenderer->SetActiveCamera( mpVtkCamera );
}

//--------------------------------------------------------------------------------------------------
void Camera::setColor( uint8_t r, uint8_t g, uint8_t b )
{
    mpVtkCameraActor->GetProperty()->SetColor( ((float)r)/255.0f, ((float)g)/255.0f, ((float)b)/255.0f );
}
