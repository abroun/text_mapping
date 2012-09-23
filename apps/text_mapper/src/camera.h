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

#ifndef CAMERA_H_
#define CAMERA_H_

//--------------------------------------------------------------------------------------------------
#include <Eigen/Core>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vector>

//--------------------------------------------------------------------------------------------------
//! Holds information about a camera, and enables us to render debug graphics representing the
//! camera
class Camera
{
    public: Camera();
    public: virtual ~Camera();

    public: void setImageSize( float imageWidth, float imageHeight );
    public: void setCameraInWorldSpaceMatrix( const Eigen::Matrix4d& cameraInWorldSpaceMatrix );
    public: void setCalibrationMatrix( const Eigen::Matrix3d& calibrationMtx );
    public: void setClipPlanes( float near, float far );

    public: void addPickPoint( const Eigen::Vector2d& screenPos );

    public: void updatePickLines();
    public: void tweakLookAtPos( const Eigen::Vector3d& offset );

    public: void showInRenderer( vtkRenderer* pRenderer );
    public: void setAsActiveCamera( vtkRenderer* pRenderer );
    public: void setColor( uint8_t r, uint8_t g, uint8_t b );

    private: float mImageWidth;
    private: float mImageHeight;

    private: Eigen::Vector3d mTotalOffset;

    private: vtkSmartPointer<vtkCamera> mpVtkCamera;
    private: vtkSmartPointer<vtkCameraActor> mpVtkCameraActor;

    private: vtkSmartPointer<vtkPolyDataMapper> mpPickLinesMapper;
    private: vtkSmartPointer<vtkActor> mpPickLinesActor;
    private: std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > mPickPoints;

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // CAMERA_H_
