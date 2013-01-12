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
#include <vtkOBJExporter.h>
#include "text_mapping/signed_distance_field.h"
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

	// Prepare to render model
	mpMarchingCubes = vtkMarchingCubes::New();
	mpModelMapper = vtkPolyDataMapper::New();
	mpModelActor = vtkActor::New();

	mpModelMapper->SetInput( mpMarchingCubes->GetOutput() );
	mpModelActor->SetMapper( mpModelMapper );

	mpRenderer->AddActor( mpModelActor );

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
void ModelViewDialog::buildModel( const PointCloudWithPoseVector& pointCloudsAndPoses, const Camera* pHighResCamera )
{
	// Clear away existing point cloud widgets
	for ( uint32_t widgetIdx = 0; widgetIdx < mPointCloudWidgets.size(); widgetIdx++ )
	{
		mpRenderer->RemoveActor( mPointCloudWidgets[ widgetIdx ].mpPointCloudActor );
	}
	mPointCloudWidgets.clear();

	// Create a point cloud widget for each point cloud
	/*for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
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
	}*/

	// Output all of the points to a text file
	FILE* pPointFile = fopen( "modelPoints.txt", "w" );

	for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
    {
	    const PointCloudWithPose& p = pointCloudsAndPoses[ i ];

	    Eigen::Vector3f normal = -p.mTransform.block<3,1>( 0, 2 );

	    for ( uint32_t pointIdx = 0; pointIdx < p.mpCloud->getNumPoints(); pointIdx++ )
	    {
	        Eigen::Vector3f pos = p.mpCloud->getPointWorldPos( pointIdx );
	        pos = p.mTransform.block<3,3>( 0, 0 )*pos + p.mTransform.block<3,1>( 0, 3 );
	        fprintf( pPointFile, "%f %f %f %f %f %f\n",
	            pos[ 0 ], pos[ 1 ], pos[ 2 ], normal[ 0 ], normal[ 1 ], normal[ 2 ] );
	    }
    }

    fclose( pPointFile );

	if ( pointCloudsAndPoses.size() > 0 )
	{
		const float VOXEL_SIDE_LENGTH = 0.001;
		const float BOX_SIZE_X = 0.2;
		const float BOX_SIZE_Y = 0.4;
		const float BOX_SIZE_Z = 0.2;

		// Locate the centre point of the first point cloud
		Eigen::Vector3f firstCorner;
		Eigen::Vector3f secondCorner;
		pointCloudsAndPoses[ 0 ].mpCloud->getBoundingBox( &firstCorner, &secondCorner );

		Eigen::Vector3f centrePos = (firstCorner + secondCorner)/2.0f;
		Eigen::Vector3i sdfDimensions( BOX_SIZE_X/VOXEL_SIDE_LENGTH,
			BOX_SIZE_Y/VOXEL_SIDE_LENGTH, BOX_SIZE_Z/VOXEL_SIDE_LENGTH );

		// Build a Signed Distance Field (SDF) from the point clouds
		SignedDistanceField signedDistanceField( centrePos, sdfDimensions, VOXEL_SIDE_LENGTH );

		for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
		{
			printf( "Adding frame %i to SDF\n", i );
			signedDistanceField.addPointCloud(
				*(pointCloudsAndPoses[ i ].mpCloud), pointCloudsAndPoses[ i ].mTransform );
		}

		signedDistanceField.outputToVTKFile( "/home/abroun/modelSDF.vtk" );

		// Transfer the SDF to image data
		mpImageData = vtkImageData::New();
		mpImageData->SetDimensions( sdfDimensions[ 0 ], sdfDimensions[ 1 ], sdfDimensions[ 2 ] );
		mpImageData->SetScalarTypeToDouble();
		mpImageData->SetNumberOfScalarComponents( 1 );
		mpImageData->AllocateScalars();

		for ( int32_t x = 0; x < sdfDimensions[ 0 ]; x++ )
		{
			for ( int32_t y = 0; y < sdfDimensions[ 1 ]; y++ )
			{
				for ( int32_t z = 0; z < sdfDimensions[ 2 ]; z++ )
				{
					*(double*)(mpImageData->GetScalarPointer( x, y, z )) = signedDistanceField.getVoxelValue( x, y, z );
				}
			}
		}

		mpMarchingCubes->SetInput( mpImageData );

		// Use a contour filter to extract a polygonal model

		// Apply texture to the model
	}

	// Update the view
	this->qvtkWidget->update();

	vtkSmartPointer<vtkOBJExporter> pExporter = vtkSmartPointer<vtkOBJExporter>::New();
	pExporter->SetRenderWindow( qvtkWidget->GetRenderWindow() );
	pExporter->SetFilePrefix( "/home/abroun/modelMC" );
	pExporter->Write();
}


