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
#include <stdint.h>
#include <algorithm>
#include <string>
#include <Eigen/Dense>
#include "text_mapping/point_cloud.h"
#include "text_mapping/signed_distance_field.h"

//--------------------------------------------------------------------------------------------------
// SignedDistanceField
//--------------------------------------------------------------------------------------------------
SignedDistanceField::SignedDistanceField( const Eigen::Vector3f& centrePos,
    const Eigen::Vector3i& dimensions, float voxelSideLength )
    : mCentrePos( centrePos ),
      mDimensions( dimensions.cwiseMax( Eigen::Vector3i::Zero() ) ),    // No -ve dimensions
      mVoxelSideLength( voxelSideLength )
{
    mVoxels.resize( mDimensions.prod(), Voxel() );
}

//--------------------------------------------------------------------------------------------------
void SignedDistanceField::addPointCloud( const PointCloud& pointCloud, const Eigen::Matrix4f& transform )
{
    const float MAX_SDF_DISTANCE = 0.01;
    const float NEAR_CLIP_Z = 0.1;

    Eigen::Matrix4f voxelInPointCloudSpaceTransform = transform.inverse();
    Eigen::Vector3f cornerPos = mCentrePos - mDimensions.cast<float>()*mVoxelSideLength/2.0;

    int32_t voxelIdx = 0;
    float curZ = cornerPos[ 2 ] + mVoxelSideLength/2.0;
    for ( int32_t z = 0; z < mDimensions[ 2 ]; z++, curZ += mVoxelSideLength )
    {
        float curY = cornerPos[ 1 ] + mVoxelSideLength/2.0;
        for ( int32_t y = 0; y < mDimensions[ 1 ]; y++, curY += mVoxelSideLength )
        {
        	float curX = cornerPos[ 0 ] + mVoxelSideLength/2.0;
            for ( int32_t x = 0; x < mDimensions[ 0 ]; x++, voxelIdx++, curX += mVoxelSideLength )
            {
                //printf( "Checking %f %f %f\n", curX, curY, curZ );
                //printf( "Value is %f\n", mVoxels[ voxelIdx ].mValue );

                Eigen::Vector4f voxelCentrePos( curX, curY, curZ, 1.0 );
                Eigen::Vector3f pointCloudPos =
                    (voxelInPointCloudSpaceTransform*voxelCentrePos).block<3,1>( 0, 0 );

                //printf( "Projected voxel %i %i %i to %f %f %f\n", x, y, z, pointCloudPos.x(), pointCloudPos.y(), pointCloudPos.z() );

                if ( pointCloudPos[ 2 ] <= NEAR_CLIP_Z )
                {
                    continue;   // Point on or behind near clip plane
                }

                Eigen::Vector2f imagePos = pointCloud.convertWorldPosToImagePos( pointCloudPos );

                //printf( "Projected voxel %i %i %i to %f %f\n", x, y, z, imagePos.x(), imagePos.y() );

                int32_t pointIdx = pointCloud.getPointIdxAtImagePos( (int32_t)imagePos.x(), (int32_t)imagePos.y() );
                if ( PointCloud::INVALID_POINT_IDX == pointIdx )
                {
                    continue;   // No point matches the voxel
                }

                //printf( "Projected voxel %i %i %i to point %i\n", x, y, z, pointIdx );

                float distanceToVoxelCentre = pointCloudPos.norm();
                float distanceToSurface = pointCloud.getPointWorldPos( pointIdx ).norm();

                float sdf = distanceToVoxelCentre - distanceToSurface;
                double tsdf = (double)std::max( -1.0f, std::min( 1.0f, sdf/MAX_SDF_DISTANCE ) );
                double pointWeight = 1.0;   // Hard-code all weights to 1.0 for now

                //printf( "%f %f %f\n", distanceToVoxelCentre, distanceToSurface, tsdf );
                if ( tsdf >= 1.0 )
                {
                    continue;   // Ignore voxels that are too far behind the surface
                    // NOTE: We could taper the weighting function rather than just cutting it off
                }

                Voxel& curVoxel = mVoxels[ voxelIdx ];
                if ( curVoxel.mWeight <= 0.0 )
                {
                    curVoxel.mValue = (pointWeight*tsdf)/pointWeight;
                    curVoxel.mWeight = pointWeight;
                }
                else
                {
                    float newWeight = curVoxel.mWeight + pointWeight;
                    curVoxel.mValue = (curVoxel.mWeight*curVoxel.mValue + pointWeight*tsdf)/newWeight;
                    curVoxel.mWeight = newWeight;
                }
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
void SignedDistanceField::outputToVTKFile( const std::string& filename ) const
{
    FILE* pVTKFile = fopen( filename.c_str(), "wb" );

    fprintf( pVTKFile, "# vtk DataFile Version 2.0\n" );
    fprintf( pVTKFile, "Signed Distance Field\n" );
    fprintf( pVTKFile, "BINARY\n" );
    fprintf( pVTKFile, "DATASET STRUCTURED_POINTS\n" );
    fprintf( pVTKFile, "DIMENSIONS %i %i %i\n", mDimensions.x(), mDimensions.y(), mDimensions.z() );
    fprintf( pVTKFile, "SPACING 1 1 1\n" );
    fprintf( pVTKFile, "ORIGIN 0 0 0\n" );
    fprintf( pVTKFile, "POINT_DATA %i\n", (uint32_t)mVoxels.size() );
    fprintf( pVTKFile, "SCALARS distance_scalars float 1\n" );
    fprintf( pVTKFile, "LOOKUP_TABLE default\n" );

    printf( "About to allocate array of %i bytes\n", 4*(uint32_t)mVoxels.size() );
    fflush( stdout );

    float* pOutputData = new float[ mVoxels.size() ];
    for ( size_t voxelIdx = 0; voxelIdx < mVoxels.size(); voxelIdx++ )
    {
        // Convert to float, swap bytes and output
        float floatValue = (float)(mVoxels[ voxelIdx ].mValue);
        char tmp;
        char* pBytes = (char*)&floatValue;
        tmp = pBytes[ 0 ];
        pBytes[ 0 ] = pBytes[ 3 ];
        pBytes[ 3 ] = tmp;
        tmp = pBytes[ 1 ];
        pBytes[ 1 ] = pBytes[ 2 ];
        pBytes[ 2 ] = tmp;

        pOutputData[ voxelIdx ] = floatValue;
    }

    fwrite( (char*)pOutputData, sizeof(float), mVoxels.size(), pVTKFile );

    fclose( pVTKFile );

    delete [] pOutputData;
}
