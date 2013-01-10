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
// File: model_view_dialog.cpp
// Desc: A dialog for viewing a model constructed from a number of frames
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include "model_view_dialog.h"
#include <Eigen/Dense>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
// ModelViewDialog
//--------------------------------------------------------------------------------------------------
ModelViewDialog::ModelViewDialog()
{
    setupUi( this );

    this->setWindowFlags( Qt::Window );

    // Set up a renderer and connect it to QT
	mpRenderer = vtkRenderer::New();
	qvtkWidget->GetRenderWindow()->AddRenderer( mpRenderer );

	mpRenderer->SetBackground( 0.0, 0.0, 0.0 );

	// Hook up signals
	connect( this->btnClose, SIGNAL( clicked() ), this, SLOT( onBtnCloseClicked() ) );
}

//--------------------------------------------------------------------------------------------------
ModelViewDialog::~ModelViewDialog()
{
}

//--------------------------------------------------------------------------------------------------
void ModelViewDialog::onBtnCloseClicked()
{
	close();
}

//--------------------------------------------------------------------------------------------------
void ModelViewDialog::buildModel( const PointCloudWithPoseVector& pointCloudsAndPoses )
{
	// Clear away existing point cloud widgets
	for ( uint32_t widgetIdx = 0; widgetIdx < mPointCloudWidgets.size(); widgetIdx++ )
	{
		mpRenderer->RemoveActor( mPointCloudWidgets[ widgetIdx ].mpPointCloudActor );
	}
	mPointCloudWidgets.clear();

	// Create a point cloud widget for each point cloud
	for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
	{
		const PointCloudWithPose& p = pointCloudsAndPoses[ i ];

		PointCloudWidget widget;

		widget.mpPointCloudSource = vtkSmartPointer<vtkPointCloudSource>::New();
		widget.mpPointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		widget.mpPointCloudActor = vtkSmartPointer<vtkActor>::New();

		widget.mpPointCloudMapper->SetInputConnection( widget.mpPointCloudSource->GetOutputPort() );
		widget.mpPointCloudSource->SetPointCloudPtr( p.mpCloud );

		widget.mpPointCloudActor->SetMapper( widget.mpPointCloudMapper );
		widget.mpPointCloudActor->GetProperty()->SetPointSize( 3.0 );

		// Break the rotation matrix down into angles Z, X, Y. The order in which VTK will apply them
		Eigen::Vector3f angles = p.mTransform.block<3,3>( 0, 0 ).eulerAngles( 2, 0, 1 );

		// Now pass the angles as degrees to VTK
		float degreesX = Utilities::radToDeg( angles[ 1 ] );
		float degreesY = Utilities::radToDeg( angles[ 2 ] );
		float degreesZ = Utilities::radToDeg( angles[ 0 ] );
		widget.mpPointCloudActor->SetOrientation( degreesX, degreesY, degreesZ );

		const Eigen::Vector3f& pos = p.mTransform.block<3,1>( 0, 3 );
		widget.mpPointCloudActor->SetPosition( pos[ 0 ], pos[ 1 ], pos[ 2 ] );

		widget.mpPointCloudActor->SetVisibility( 1 );

		mpRenderer->AddActor( widget.mpPointCloudActor );

		mPointCloudWidgets.push_back( widget );
	}

	// Update the view
	this->qvtkWidget->update();
}


