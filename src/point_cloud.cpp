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
#include <stdio.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include "text_mapping/point_cloud.h"

//--------------------------------------------------------------------------------------------------
// PointCloud
//--------------------------------------------------------------------------------------------------
PointCloud::PointCloud()
	: mFocalLengthMM( 1000.0 ),
	  mImage( 1, 1, CV_8UC4 )
{
	mPointMap.resize( mImage.rows*mImage.cols, INVALID_POINT_IDX );
}

//--------------------------------------------------------------------------------------------------
PointCloud::PointCloud( uint32_t width, uint32_t height, float focalLengthMM )
	: mFocalLengthMM( focalLengthMM ),
	  mImage( height, width, CV_8UC4 )
{
	mPointMap.resize( mImage.rows*mImage.cols, INVALID_POINT_IDX );
}

//--------------------------------------------------------------------------------------------------
PointCloud::~PointCloud()
{

}

//--------------------------------------------------------------------------------------------------
PointCloud::Ptr PointCloud::loadTextMapFromSpcFile( const std::string& filename )
{
	Ptr pResult;

	FILE* pSpcFile = fopen( filename.c_str(), "r" );
	if ( NULL != pSpcFile )
	{

#define MAX_TOKEN_SIZE 256
#define NUMBER( n ) #n
#define TOKEN_FORMAT_STRING( a ) " %" NUMBER( a ) "s "

		char tokenBuffer[ MAX_TOKEN_SIZE + 1 ];

		bool bGotWidth = false;
		bool bGotHeight = false;
		bool bGotFocalLength = false;

		uint32_t width = 0;
		uint32_t height = 0;
		float focalLengthMM = 0.0f;

		bool bFinished = false;

		// Read out information to construct the point cloud
		while ( !( bGotWidth && bGotHeight && bGotFocalLength ) && !bFinished )
		{
			int readResult = fscanf( pSpcFile, TOKEN_FORMAT_STRING( MAX_TOKEN_SIZE ), tokenBuffer );
			if ( EOF == readResult )
			{
				bFinished = true;
			}
			else
			{
				// Parse the token
				if ( boost::iequals( tokenBuffer, "WIDTH" ) )
				{
					readResult = fscanf( pSpcFile, " %u ", &width );
					bGotWidth = true;
				}
				else if ( boost::iequals( tokenBuffer, "HEIGHT" ) )
				{
					readResult = fscanf( pSpcFile, " %u ", &height );
					bGotHeight = true;
				}
				else if ( boost::iequals( tokenBuffer, "FOCAL_LENGTH_MM" ) )
				{
					readResult = fscanf( pSpcFile, " %f ", &focalLengthMM );
					bGotFocalLength = true;
				}
			}
		}

		if ( bGotWidth && bGotHeight && bGotFocalLength )
		{
			// We got enough info to construct the point cloud
			Ptr pPointCloud( new PointCloud( width, height, focalLengthMM ) );

			float imageCentreX = (float)(width/2) - 0.5f;
			float imageCentreY = (float)(height/2) - 0.5f;

			// Now read in depth data and, if it exists, RGBA data
			bool bBinaryData = false;
			bool bGotDepthData = false;
			bFinished = false;

			std::vector<float> depthBuffer;
			depthBuffer.resize( width*height );

			while ( !bFinished )
			{
				int readResult = fscanf( pSpcFile, TOKEN_FORMAT_STRING( MAX_TOKEN_SIZE ), tokenBuffer );
				if ( EOF == readResult )
				{
					bFinished = true;
				}
				else
				{
					// Parse the token
					if ( boost::iequals( tokenBuffer, "BINARY" ) )
					{
						int useBinaryData;
						readResult = fscanf( pSpcFile, " %i ", &useBinaryData );
						bBinaryData = useBinaryData;
					}
					else if ( boost::iequals( tokenBuffer, "DEPTH_DATA" ) )
					{
						if ( bBinaryData )
						{
							readResult = fread( &depthBuffer[ 0 ], sizeof( float ), width*height, pSpcFile );
							bGotDepthData = true;
						}
						else
						{
							fprintf( stderr, "Error: ASCII depth data not supported yet\n" );
						}
					}
					else if ( boost::iequals( tokenBuffer, "RGBA_DATA" ) )
					{
						if ( bBinaryData )
						{
							readResult = fread( pPointCloud->mImage.data, sizeof( uint8_t ), width*height*4, pSpcFile );
						}
						else
						{
							fprintf( stderr, "Error: ASCII RGBA data not supported yet\n" );
						}
					}
				}
			}

			// Construct the point cloud if we got depth data
			if ( bGotDepthData )
			{
				for ( uint32_t y = 0; y < height; y++ )
				{
					for ( uint32_t x = 0; x < width; x++ )
					{
						uint32_t pixelIdx = y*width + x;

						float depthValue = depthBuffer[ pixelIdx ];
						if ( !std::isnan( depthValue ) )
						{
							// We've found a valid point
							pPointCloud->mPointMap[ pixelIdx ] = pPointCloud->mPointWorldPositions.size();

							pPointCloud->mPointWorldPositions.push_back( Eigen::Vector3f(
								((float)x - imageCentreX)/1000.0f,
								((float)y - imageCentreY)/1000.0f,
								focalLengthMM/1000.0f ) );
						}
					}
				}

				pResult = pPointCloud;
			}
			else
			{
				fprintf( stderr, "Warning: No depth data found for point cloud\n" );
			}
		}
		else
		{
			fprintf( stderr, "Warning: Not enough info to construct point cloud\n" );
		}
	}

	return pResult;
}

//--------------------------------------------------------------------------------------------------
int32_t PointCloud::getPointIdxAtImagePos( int32_t u, int32_t v ) const
{
	int32_t result = INVALID_POINT_IDX;

	if ( u >= 0 && u < mImage.cols
		&& v >= 0 && v < mImage.rows )
	{
		result = mPointMap[ v*mImage.cols + u ];
	}

	return result;
}

//--------------------------------------------------------------------------------------------------
Eigen::Vector2f PointCloud::getPointImagePos( int32_t pointIdx ) const
{
	if ( pointIdx < 0 || pointIdx >= (int32_t)mPointWorldPositions.size() )
	{
		// Return the zero vector for invalid points
		return Eigen::Vector2f::Zero();
	}

	float imageCentreX = (float)(mImage.cols/2) - 0.5f;
	float imageCentreY = (float)(mImage.rows/2) - 0.5f;

	const Eigen::Vector3f pos = mPointWorldPositions[ pointIdx ];

	return Eigen::Vector2f(
		imageCentreX + mFocalLengthMM*pos[ 0 ]/pos[ 2 ],
		imageCentreY + mFocalLengthMM*pos[ 1 ]/pos[ 2 ] );
}


