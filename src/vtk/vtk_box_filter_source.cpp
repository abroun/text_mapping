/*

Copyright (c) 2013, Bristol Robotics Laboratory
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
#include <assert.h>
#include <math.h>

#include <vtkCellArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkLine.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <Eigen/Core>

#include "text_mapping/vtk/vtk_box_filter_source.h"

//--------------------------------------------------------------------------------------------------
// vtkBoxFilterSource
//--------------------------------------------------------------------------------------------------
vtkStandardNewMacro( vtkBoxFilterSource );

//--------------------------------------------------------------------------------------------------
vtkBoxFilterSource::vtkBoxFilterSource()
{
    this->SetNumberOfInputPorts( 0 );
}

//--------------------------------------------------------------------------------------------------
int vtkBoxFilterSource::RequestData(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector )
{
    // Get the info object
    vtkInformation* outInfo = outputVector->GetInformationObject( 0 );

    // Get the output
    vtkPolyData* output = vtkPolyData::SafeDownCast( outInfo->Get( vtkDataObject::DATA_OBJECT() ) );

    int piece = outInfo->Get( vtkStreamingDemandDrivenPipeline::UPDATE_PIECE_NUMBER() );
    int numPieces = outInfo->Get( vtkStreamingDemandDrivenPipeline::UPDATE_NUMBER_OF_PIECES() );

    // For now, we hack around the streaming pipeline, and just return 1 piece
    if ( numPieces > 1 )
    {
        numPieces = 1;
    }
    if ( piece >= numPieces )
    {
        // Although the super class should take care of this,
        // it cannot hurt to check here.
        return 1;
    }

    // Create the PolyData for the BoxFilter
    vtkSmartPointer<vtkPoints> pPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> pLines = vtkSmartPointer<vtkCellArray>::New();

    // Create the points
    std::vector<Eigen::Vector3f> cornerPoints = this->mBoxFilter.calculateCorners();

    for ( uint32_t cornerIdx = 0; cornerIdx < cornerPoints.size(); cornerIdx++ )
    {
        pPoints->InsertNextPoint( cornerPoints[ cornerIdx ].data() );
    }

    // Create the lines
    vtkIdType pointIndices[] =
    {
        0, 1,       // Top left
        1, 2,       // Top back
        2, 3,       // Top right
        3, 0,       // Top front
        4, 5,       // Bottom left
        5, 6,       // Bottom back
        6, 7,       // Bottom right
        7, 4,       // Bottom front
        0, 4,       // Front left
        1, 5,       // Back left
        2, 6,       // Back right
        3, 7,       // Front right
    };

    for ( int32_t lineIdx = 0; lineIdx < 12; lineIdx++ )
    {
        vtkSmartPointer<vtkLine> pLine = vtkSmartPointer<vtkLine>::New();
        pLine->GetPointIds()->SetId( 0, pointIndices[ 2*lineIdx ] );
        pLine->GetPointIds()->SetId( 1, pointIndices[ 2*lineIdx + 1 ] );

        pLines->InsertNextCell( pLine );
    }

    output->SetPoints( pPoints );
    output->SetLines( pLines );

    return 1;
}

//--------------------------------------------------------------------------------------------------
void vtkBoxFilterSource::PrintSelf( std::ostream& os, vtkIndent indent )
{
    this->Superclass::PrintSelf( os, indent );

    os << indent << "Transform:\n" << this->mBoxFilter.mTransform << "\n"
        << "Dimensions:\n" << this->mBoxFilter.mDimensions << "\n";
}

//--------------------------------------------------------------------------------------------------
int vtkBoxFilterSource::RequestInformation(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector)
{
    // get the info object
    vtkInformation* outInfo = outputVector->GetInformationObject(0);

    outInfo->Set( vtkStreamingDemandDrivenPipeline::MAXIMUM_NUMBER_OF_PIECES(), 1 );

    // Calculate the bounding box
    std::vector<Eigen::Vector3f> cornerPoints = this->mBoxFilter.calculateCorners();

    Eigen::Vector3f firstCorner = cornerPoints[ 0 ];
    Eigen::Vector3f secondCorner = cornerPoints[ 0 ];

    for ( uint32_t cornerIdx = 1; cornerIdx < cornerPoints.size(); cornerIdx++ )
    {
        firstCorner[ 0 ] = std::min( firstCorner[ 0 ], cornerPoints[ cornerIdx ][ 0 ] );
        firstCorner[ 1 ] = std::min( firstCorner[ 1 ], cornerPoints[ cornerIdx ][ 1 ] );
        firstCorner[ 2 ] = std::min( firstCorner[ 2 ], cornerPoints[ cornerIdx ][ 2 ] );

        secondCorner[ 0 ] = std::max( secondCorner[ 0 ], cornerPoints[ cornerIdx ][ 0 ] );
        secondCorner[ 1 ] = std::max( secondCorner[ 1 ], cornerPoints[ cornerIdx ][ 1 ] );
        secondCorner[ 2 ] = std::max( secondCorner[ 2 ], cornerPoints[ cornerIdx ][ 2 ] );
    }

    outInfo->Set( vtkStreamingDemandDrivenPipeline::WHOLE_BOUNDING_BOX(),
        firstCorner[ 0 ], secondCorner[ 0 ],       // x
        firstCorner[ 1 ], secondCorner[ 1 ],       // y
        firstCorner[ 2 ], secondCorner[ 2 ] );     // z

    return 1;
}



