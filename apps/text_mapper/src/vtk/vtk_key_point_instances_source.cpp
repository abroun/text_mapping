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
#include "vtkCellArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtk_key_point_instances_source.h"

//--------------------------------------------------------------------------------------------------
// vtkKeyPointInstancesSource
//--------------------------------------------------------------------------------------------------
vtkStandardNewMacro( vtkKeyPointInstancesSource );

//--------------------------------------------------------------------------------------------------
vtkKeyPointInstancesSource::vtkKeyPointInstancesSource()
{
    this->SetNumberOfInputPorts( 0 );
}

//--------------------------------------------------------------------------------------------------
int vtkKeyPointInstancesSource::RequestData(
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

    if ( this->KeyPointInstances.size() > 0 )
    {
        vtkPoints* pNewPoints;
        vtkUnsignedCharArray* pNewColours;
        vtkCellArray* pNewCells;

        uint32_t numPoints = this->KeyPointInstances.size();

        pNewPoints = vtkPoints::New();
        pNewPoints->Allocate( numPoints );
        pNewColours = vtkUnsignedCharArray::New();
        pNewColours->SetNumberOfComponents( 3 );
        pNewColours->Allocate( 3*numPoints );
        pNewColours->SetName( "Colors" );

        pNewCells = vtkCellArray::New();

        for ( uint32_t pointIdx = 0; pointIdx < numPoints; pointIdx++ )
        {
            // Add vertex
            Eigen::Vector3d pointPos = this->KeyPointInstances[ pointIdx ].mKeyPointInstance.mPos.cast<double>();
            pNewPoints->InsertNextPoint( pointPos.data() );

            // Add color
            uint8_t color[ 3 ] =
            {
                this->KeyPointInstances[ pointIdx ].mR,
                this->KeyPointInstances[ pointIdx ].mG,
                this->KeyPointInstances[ pointIdx ].mB
            };
            pNewColours->InsertNextTupleValue( color );

            pNewCells->InsertNextCell( 1 );
            pNewCells->InsertCellPoint( pointIdx );
        }

        // Update ourselves and release memory
        pNewPoints->Squeeze();
        output->SetPoints( pNewPoints );
        pNewPoints->Delete();

        pNewCells->Squeeze();
        output->SetVerts( pNewCells );
        pNewCells->Delete();

        pNewColours->Squeeze();
        output->GetPointData()->SetScalars( pNewColours );
        pNewColours->Delete();
    }

    return 1;
}

//--------------------------------------------------------------------------------------------------
void vtkKeyPointInstancesSource::PrintSelf( std::ostream& os, vtkIndent indent )
{
    this->Superclass::PrintSelf( os, indent );

    os << indent << "Num Key Point Instance: " << this->KeyPointInstances.size() << "\n";
}

//--------------------------------------------------------------------------------------------------
int vtkKeyPointInstancesSource::RequestInformation(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector)
{
    // get the info object
    vtkInformation* outInfo = outputVector->GetInformationObject(0);

    outInfo->Set( vtkStreamingDemandDrivenPipeline::MAXIMUM_NUMBER_OF_PIECES(), 1 );

    if ( this->KeyPointInstances.size() <= 0 )
    {
        outInfo->Set( vtkStreamingDemandDrivenPipeline::WHOLE_BOUNDING_BOX(),
           -1.0, 1.0,       // x
           -1.0, 1.0,       // y
           -1.0, 1.0 );     // z
    }
    else
    {
        // Calculate the bounding box
        Eigen::Vector3f firstCorner = this->KeyPointInstances[ 0 ].mKeyPointInstance.mPos;
        Eigen::Vector3f secondCorner = this->KeyPointInstances[ 0 ].mKeyPointInstance.mPos;

        for ( uint32_t instanceIdx = 1; instanceIdx < KeyPointInstances.size(); instanceIdx++ )
        {
            const Eigen::Vector3f& p = this->KeyPointInstances[ instanceIdx ].mKeyPointInstance.mPos;

            if ( p[ 0 ] < firstCorner[ 0 ] ) firstCorner[ 0 ] = p[ 0 ];
            if ( p[ 1 ] < firstCorner[ 1 ] ) firstCorner[ 1 ] = p[ 1 ];
            if ( p[ 2 ] < firstCorner[ 2 ] ) firstCorner[ 2 ] = p[ 2 ];

            if ( p[ 0 ] > secondCorner[ 0 ] ) secondCorner[ 0 ] = p[ 0 ];
            if ( p[ 1 ] > secondCorner[ 1 ] ) secondCorner[ 1 ] = p[ 1 ];
            if ( p[ 2 ] > secondCorner[ 2 ] ) secondCorner[ 2 ] = p[ 2 ];
        }

        outInfo->Set( vtkStreamingDemandDrivenPipeline::WHOLE_BOUNDING_BOX(),
            firstCorner[ 0 ], secondCorner[ 0 ],       // x
            firstCorner[ 1 ], secondCorner[ 1 ],       // y
            firstCorner[ 2 ], secondCorner[ 2 ] );     // z
    }

    return 1;
}






