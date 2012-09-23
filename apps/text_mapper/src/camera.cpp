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
#include <vtkMatrix4x4.h>
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
    mpVtkCameraActor->SetWidthByHeightRatio( 4.0/3.0 );
}

//--------------------------------------------------------------------------------------------------
Camera::~Camera()
{
}

//--------------------------------------------------------------------------------------------------
void Camera::setCameraInWorldSpaceMatrix( const Eigen::Matrix4f& cameraInWorldSpaceMatrix )
{
    Eigen::Matrix4d m = cameraInWorldSpaceMatrix.cast<double>();

    //Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    //m.block<3,1>( 0, 3 ) = Eigen::Vector3d( 0.0, 0.0, -2.0 );

    //mpVtkCamera->GetViewTransformMatrix()->DeepCopy( m.data() );

    mpVtkCamera->SetPosition( 0.0, 0.0, 0.0 );
    mpVtkCamera->SetFocalPoint( 0.0, 0.0, 1.0 );
    mpVtkCamera->SetViewUp( 0.0, -1.0, 0.0 );
}

//--------------------------------------------------------------------------------------------------
void Camera::setCalibrationMatrix( const Eigen::Matrix3d& calibrationMtx, float halfImageHeight )
{
    float halfViewAngle = Utilities::radToDeg(
        atan2f( halfImageHeight, calibrationMtx( 1, 1 ) ) );

    printf( "f_y %f and half height %f\n", calibrationMtx( 1, 1 ), halfImageHeight );
    printf( "Half view angle is %f degrees\n", halfViewAngle );

    mpVtkCamera->SetViewAngle( 2.0*halfViewAngle );
    mpVtkCamera->SetClippingRange( calibrationMtx( 1, 1 )/1000.0, 2.0 );
}

//--------------------------------------------------------------------------------------------------
void Camera::showInRenderer( vtkRenderer* pRenderer )
{
    pRenderer->AddActor( mpVtkCameraActor );
}
