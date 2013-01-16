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
// File: model_view_dialog.h
// Desc: A dialog for viewing a model constructed from a number of frames
//--------------------------------------------------------------------------------------------------

#ifndef MODEL_VIEW_DIALOG_H_
#define MODEL_VIEW_DIALOG_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <QtGui/QDialog>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include "text_mapping/point_cloud.h"
#include "text_mapping/vtk/vtk_point_cloud_source.h"
#include "../camera.h"
#include "ui_model_view_dialog.h"

//--------------------------------------------------------------------------------------------------
struct PointCloudWithPose
{
	PointCloudWithPose() : mFrameIdx( 0 ) {}
	PointCloudWithPose( PointCloud::Ptr pCloud, const Eigen::Matrix4f& transform,
	                    cv::Mat highResImage, uint32_t frameIdx )
		: mpCloud( pCloud ), mTransform( transform ),
		  mHighResImage( highResImage ), mFrameIdx( frameIdx ) {}

	PointCloud::Ptr mpCloud;
	Eigen::Matrix4f mTransform;
	cv::Mat mHighResImage;
	uint32_t mFrameIdx;

	public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION( PointCloudWithPose );

typedef std::vector<PointCloudWithPose, Eigen::aligned_allocator<PointCloudWithPose> > PointCloudWithPoseVector;

//--------------------------------------------------------------------------------------------------
struct PointCloudWidget
{
	vtkSmartPointer<vtkPointCloudSource> mpPointCloudSource;
    vtkSmartPointer<vtkPolyDataMapper> mpPointCloudMapper;
    vtkSmartPointer<vtkActor> mpPointCloudActor;
};

//--------------------------------------------------------------------------------------------------
class ModelViewDialog : public QDialog, private Ui::model_view_dialog
{
    Q_OBJECT

    public: ModelViewDialog();
    public: virtual ~ModelViewDialog();

    public slots: void onBtnCloseClicked();

	public: void buildModel( const PointCloudWithPoseVector& pointCloudsAndPoses, const Camera* pHighResCamera );

	private: vtkSmartPointer<vtkRenderer> mpRenderer;
	private: std::vector<PointCloudWidget> mPointCloudWidgets;

	private: vtkSmartPointer<vtkImageData> mpImageData;
	private: vtkSmartPointer<vtkMarchingCubes> mpMarchingCubes;
	private: vtkSmartPointer<vtkPolyDataMapper> mpModelMapper;
	private: vtkSmartPointer<vtkActor> mpModelActor;
};

#endif // MODEL_VIEW_DIALOG_H_
