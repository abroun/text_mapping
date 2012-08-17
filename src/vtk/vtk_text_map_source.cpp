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
#include <assert.h>
#include <math.h>

#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include <Eigen/Core>

#include "text_mapping/vtk/vtk_text_map_source.h"

//--------------------------------------------------------------------------------------------------
// vtkTextMapSource
//--------------------------------------------------------------------------------------------------
vtkStandardNewMacro( vtkTextMapSource );

//--------------------------------------------------------------------------------------------------
vtkTextMapSource::vtkTextMapSource()
{
    this->SetNumberOfInputPorts( 0 );
}

//--------------------------------------------------------------------------------------------------
int vtkTextMapSource::RequestData(
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

    if ( NULL != this->TextMapPtr )
    {
        vtkPoints* pNewPoints;
        vtkUnsignedCharArray* pNewColours;
        vtkFloatArray* pNewTextureCoords;
        vtkCellArray* pNewPolys;

        uint32_t numLetters = this->TextMapPtr->getNumLetters();
        uint32_t numPoints = numLetters*4;
        uint32_t numPolys = numLetters;;

        pNewPoints = vtkPoints::New();
        pNewPoints->Allocate( numPoints );
        pNewColours = vtkUnsignedCharArray::New();
        pNewColours->SetNumberOfComponents( 3 );
        pNewColours->Allocate( 3*numPoints );
        pNewColours->SetName( "Colors" );
        pNewTextureCoords = vtkFloatArray::New();
        pNewTextureCoords->SetNumberOfComponents( 2 );
        pNewTextureCoords->Allocate( 2*numPoints );
        
        pNewPolys = vtkCellArray::New();
        pNewPolys->Allocate( pNewPolys->EstimateSize( numPolys, 4 ) );

        for ( uint32_t letterIdx = 0; letterIdx < numLetters; letterIdx++ )
        {
            const Letter& letter = this->TextMapPtr->getLetter( letterIdx );

            Eigen::Vector3d vertices[] =
            {
                letter.getTopLeftPos().cast<double>(),
                letter.getTopRightPos().cast<double>(),
                letter.getBottomRightPos().cast<double>(),
                letter.getBottomLeftPos().cast<double>()
            };

            // Add vertices
            pNewPoints->InsertNextPoint( vertices[ 0 ].data() );
            pNewPoints->InsertNextPoint( vertices[ 1 ].data() );
            pNewPoints->InsertNextPoint( vertices[ 2 ].data() );
            pNewPoints->InsertNextPoint( vertices[ 3 ].data() );

            // Add colours
            uint8_t colour[ 3 ] = { 255, 255, 255 };

            pNewColours->InsertNextTupleValue( colour );
            pNewColours->InsertNextTupleValue( colour );
            pNewColours->InsertNextTupleValue( colour );
            pNewColours->InsertNextTupleValue( colour );

            // Add in texture coordinates
            float leftU = ((float)letter.mCharacter)*1.0/128.0;
            float rightU = leftU + 1.0/128.0;
            
            float textureCoords[ 4 ][ 2 ] =
            {
                { leftU, 1.0 },
                { rightU, 1.0 },
                { rightU, 0.0 },
                { leftU, 0.0 }
            };
            
            pNewTextureCoords->InsertNextTupleValue( textureCoords[ 0 ] );
            pNewTextureCoords->InsertNextTupleValue( textureCoords[ 1 ] );
            pNewTextureCoords->InsertNextTupleValue( textureCoords[ 2 ] );
            pNewTextureCoords->InsertNextTupleValue( textureCoords[ 3 ] );
            
            // Create a polygon for the letter
            vtkIdType baseIdx = 4*letterIdx;
            vtkIdType pointIndices[] = { baseIdx, baseIdx + 1, baseIdx + 2, baseIdx + 3 };
            pNewPolys->InsertNextCell( 4, pointIndices );
        }

        // Update ourselves and release memory
        pNewPoints->Squeeze();
        output->SetPoints( pNewPoints );
        pNewPoints->Delete();

        pNewColours->Squeeze();
        output->GetPointData()->SetScalars( pNewColours );
        pNewColours->Delete();

        pNewTextureCoords->Squeeze();
        output->GetPointData()->SetTCoords( pNewTextureCoords );
        pNewTextureCoords->Delete();
        
        pNewPolys->Squeeze();
        output->SetPolys( pNewPolys );
        pNewPolys->Delete();
    }

    return 1;
}

//--------------------------------------------------------------------------------------------------
void vtkTextMapSource::PrintSelf( std::ostream& os, vtkIndent indent )
{
    this->Superclass::PrintSelf( os, indent );

    if ( NULL == this->TextMapPtr )
    {
        os << indent << "Text Map: NULL\n";
    }
    else
    {
        os << indent << "Num Letters: " << this->TextMapPtr->getNumLetters() << "\n";
    }
}

//--------------------------------------------------------------------------------------------------
int vtkTextMapSource::RequestInformation(
    vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
    vtkInformationVector* outputVector)
{
    // get the info object
    vtkInformation* outInfo = outputVector->GetInformationObject(0);

    outInfo->Set( vtkStreamingDemandDrivenPipeline::MAXIMUM_NUMBER_OF_PIECES(), 1 );

    if ( NULL == this->TextMapPtr )
    {
        outInfo->Set( vtkStreamingDemandDrivenPipeline::WHOLE_BOUNDING_BOX(),
           -1.0, 1.0,       // x
           -1.0, 1.0,       // y
           -1.0, 1.0 );     // z
    }
    else
    {
        Eigen::Vector3f firstCorner;
        Eigen::Vector3f secondCorner;

        this->TextMapPtr->getBoundingBox( &firstCorner, &secondCorner );

        outInfo->Set( vtkStreamingDemandDrivenPipeline::WHOLE_BOUNDING_BOX(),
            firstCorner[ 0 ], secondCorner[ 0 ],       // x
            firstCorner[ 1 ], secondCorner[ 1 ],       // y
            firstCorner[ 2 ], secondCorner[ 2 ] );     // z
    }

    return 1;
}



