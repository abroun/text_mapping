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
#include <math.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include "text_mapping/point_cloud.h"

const float DEPTH_SCALE = 8.0;  // TODO: Check to see if this can be removed...

//--------------------------------------------------------------------------------------------------
// PointCloud
//--------------------------------------------------------------------------------------------------
PointCloud::PointCloud()
	: mFocalLengthPixels( 1000.0 )
{
    mImage = cv::Mat::zeros( 1, 1, CV_8UC4 );
	mPointMap.resize( mImage.rows*mImage.cols, INVALID_POINT_IDX );
}

//--------------------------------------------------------------------------------------------------
PointCloud::PointCloud( uint32_t width, uint32_t height, float focalLengthPixels )
	: mFocalLengthPixels( focalLengthPixels )
{
    mImage = cv::Mat::zeros( height, width, CV_8UC4 );
	mPointMap.resize( mImage.rows*mImage.cols, INVALID_POINT_IDX );
}

//--------------------------------------------------------------------------------------------------
PointCloud::~PointCloud()
{

}

//--------------------------------------------------------------------------------------------------
PointCloud::Ptr PointCloud::loadPointCloudFromSpcFile( const std::string& filename )
{
	Ptr pResult;

	FILE* pSpcFile = fopen( filename.c_str(), "rb" );
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
		float focalLengthPixels = 0.0f;

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
				else if ( boost::iequals( tokenBuffer, "FOCAL_LENGTH_PIXELS" ) )
				{
					readResult = fscanf( pSpcFile, " %f ", &focalLengthPixels );
					bGotFocalLength = true;
				}
			}
		}

		if ( bGotWidth && bGotHeight && bGotFocalLength )
		{
			// We got enough info to construct the point cloud
			Ptr pPointCloud( new PointCloud( width, height, focalLengthPixels ) );

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
						bBinaryData = (useBinaryData != 0);
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
						if ( !boost::math::isnan( depthValue ) )
						{
                            depthValue /= DEPTH_SCALE;
							
							// We've found a valid point
							pPointCloud->mPointMap[ pixelIdx ] = pPointCloud->mPointWorldPositions.size();

							pPointCloud->mPointWorldPositions.push_back( Eigen::Vector3f(
								depthValue*((float)x - imageCentreX)/focalLengthPixels,
								depthValue*((float)y - imageCentreY)/focalLengthPixels,
								depthValue ) );
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
void PointCloud::saveToSpcFile( const std::string& filename, bool bBinary )
{
    FILE* pSpcFile = fopen( filename.c_str(), "wb" );

    if ( NULL != pSpcFile )
    {
        uint32_t width = mImage.cols;
        uint32_t height = mImage.rows;

        fprintf( pSpcFile, "WIDTH %u\n", width );
        fprintf( pSpcFile, "HEIGHT %u\n", height );
        fprintf( pSpcFile, "FOCAL_LENGTH_PIXELS %f\n", mFocalLengthPixels );
        fprintf( pSpcFile, "BINARY %i\n", (int32_t)bBinary );

        // Write out depth data
        fprintf( pSpcFile, "DEPTH_DATA\n" );
        if ( bBinary )
        {
            std::vector<float> depthBuffer;
            depthBuffer.resize( width*height );
            float* pCurDepth = &depthBuffer[ 0 ];

            for ( uint32_t v = 0; v < height; v++ )
            {
                for ( uint32_t u = 0; u < width; u++ )
                {
                    int32_t pointIdx = mPointMap[ v*width + u ];
                    if ( INVALID_POINT_IDX != pointIdx )
                    {
                        *pCurDepth = mPointWorldPositions[ pointIdx ][ 2 ] * DEPTH_SCALE;
                    }
                    else
                    {
                        *pCurDepth = std::numeric_limits<float>::quiet_NaN();
                    }

                    pCurDepth++;
                }
            }

            fwrite( &depthBuffer[ 0 ], sizeof( float ), width*height, pSpcFile );
        }
        else
        {
            for ( uint32_t v = 0; v < height; v++ )
            {
                for ( uint32_t u = 0; u < width; u++ )
                {
                    int32_t pointIdx = mPointMap[ v*width + u ];
                    if ( INVALID_POINT_IDX != pointIdx )
                    {
                        fprintf( pSpcFile, "%f\n", mPointWorldPositions[ pointIdx ][ 2 ] * DEPTH_SCALE );
                    }
                    else
                    {
                        fprintf( pSpcFile, "nan\n" );
                    }
                }
            }
        }

        // Write out rgba data
        fprintf( pSpcFile, "RGBA_DATA\n" );
        if ( bBinary )
        {
            fwrite( mImage.data, sizeof( uint8_t ), width*height*4, pSpcFile );
        }
        else
        {
            for ( uint32_t v = 0; v < height; v++ )
            {
                for ( uint32_t u = 0; u < width; u++ )
                {
                    cv::Vec4b& pixelData = mImage.at<cv::Vec4b>( v, u );

                    fprintf( pSpcFile, "%i %i %i %i\n",
                        pixelData[ 0 ], pixelData[ 1 ], pixelData[ 2 ], pixelData[ 3 ] );
                }
            }
        }

        fclose( pSpcFile );
    }
    else
    {
        throw std::runtime_error( "Unable to open file" );
    }
}

//--------------------------------------------------------------------------------------------------
PointCloud::Ptr PointCloud::filterOutPointsFarFromPointSet(
	const std::vector<Eigen::Vector3f>& filterPoints, float filterDistance ) const
{
	Ptr pPointCloud( new PointCloud( mImage.cols, mImage.rows, mFocalLengthPixels ) );

	float squaredFilterDistance = filterDistance*filterDistance;
	for ( int32_t pointIdx = 0; pointIdx < (int32_t)mPointWorldPositions.size(); pointIdx++ )
	{
		// Check to see if the point is close enough to one of the filter points
		bool bPointCloseEnough = false;

		const Eigen::Vector3f& pointWorldPos = mPointWorldPositions[ pointIdx ];
		for ( uint32_t filterPointIdx = 0; filterPointIdx < filterPoints.size(); filterPointIdx++ )
		{
			if ( (pointWorldPos - filterPoints[ filterPointIdx ]).squaredNorm() <= squaredFilterDistance )
			{
				bPointCloseEnough = true;
				break;
			}
		}

		// If the point was close enough, pass it through to the new point cloud
		if ( bPointCloseEnough )
		{
			Eigen::Vector2i pointImagePos = getPointImagePos( pointIdx ).cast<int>();

			// Add the point world position
			uint32_t newPointIdx = pPointCloud->mPointWorldPositions.size();
			pPointCloud->mPointWorldPositions.push_back( pointWorldPos );

			// Update the new point map
			uint32_t pixelIdx = pointImagePos[ 1 ]*mImage.cols + pointImagePos[ 0 ];
			pPointCloud->mPointMap[ pixelIdx ] = (int32_t)newPointIdx;

			// Copy over the point's pixel
			pPointCloud->mImage.at<uint32_t>( pointImagePos[ 1 ], pointImagePos[ 0 ] ) =
			    mImage.at<uint32_t>( pointImagePos[ 1 ], pointImagePos[ 0 ] );
		}
	}

	return pPointCloud;
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

	return convertWorldPosToImagePos( mPointWorldPositions[ pointIdx ] );
}

//--------------------------------------------------------------------------------------------------
Eigen::Vector2f PointCloud::convertWorldPosToImagePos( const Eigen::Vector3f& worldPos ) const
{
	float imageCentreX = (float)(mImage.cols/2) - 0.5f;
	float imageCentreY = (float)(mImage.rows/2) - 0.5f;

	return Eigen::Vector2f(
		imageCentreX + mFocalLengthPixels*worldPos[ 0 ]/worldPos[ 2 ],
		imageCentreY + mFocalLengthPixels*worldPos[ 1 ]/worldPos[ 2 ] );
}

//--------------------------------------------------------------------------------------------------
void PointCloud::getPointColor( int32_t pointIdx,
		uint8_t* pRedOut, uint8_t* pGreenOut, uint8_t* pBlueOut, uint8_t* pAlphaOut ) const
{
	Eigen::Vector2i imagePos = getPointImagePos( pointIdx ).cast<int>();

	if ( imagePos[ 0 ] >= 0 && imagePos[ 0 ] < mImage.cols
		&& imagePos[ 1 ] >= 0 && imagePos[ 1 ] < mImage.rows )
	{
		uint8_t* pPixel = mImage.data + 4*( imagePos[ 1 ]*mImage.cols + imagePos[ 0 ] );

		*pRedOut = pPixel[ 2 ];
		*pGreenOut = pPixel[ 1 ];
		*pBlueOut = pPixel[ 0 ];
		*pAlphaOut = pPixel[ 3 ];
	}
	else
	{
		*pRedOut = 255;
		*pGreenOut = 0;
		*pBlueOut = 0;
		*pAlphaOut = 0;
	}
}

//--------------------------------------------------------------------------------------------------
int32_t PointCloud::addPoint( const Eigen::Vector3f& worldPos, uint8_t r, uint8_t g, uint8_t b, uint8_t a )
{
    int32_t pointIdx = INVALID_POINT_IDX;

    float imageCentreX = (float)(mImage.cols/2) - 0.5f;
    float imageCentreY = (float)(mImage.rows/2) - 0.5f;

    int32_t pixelX = (int32_t)(imageCentreX + mFocalLengthPixels*-worldPos[ 0 ]/worldPos[ 2 ]);
    int32_t pixelY = (int32_t)(imageCentreY + mFocalLengthPixels*-worldPos[ 1 ]/worldPos[ 2 ]);

    if ( pixelX >= 0 && pixelX <= mImage.cols
        && pixelY >= 0 && pixelY <= mImage.rows )
    {
        int32_t existingPointIdx = mPointMap[ pixelY*mImage.cols + pixelX ];

        if ( INVALID_POINT_IDX != existingPointIdx )
        {
            // Reuse the existing point
            pointIdx = existingPointIdx;
            mPointWorldPositions[ pointIdx ] = worldPos;
        }
        else
        {
            // Create a new point
            pointIdx = mPointWorldPositions.size();
            mPointWorldPositions.push_back( worldPos );
            mPointMap[ pixelY*mImage.cols + pixelX ] = pointIdx;
        }

        // Set the pixel colour
        uint8_t* pPixel = mImage.data + 4*( pixelY*mImage.cols + pixelX );

        pPixel[ 2 ] = r;
        pPixel[ 1 ] = g;
        pPixel[ 0 ] = b;
        pPixel[ 3 ] = a;
    }

    return pointIdx;
}

//--------------------------------------------------------------------------------------------------
void PointCloud::getBoundingBox( Eigen::Vector3f* pFirstCornerOut, Eigen::Vector3f* pSecondCornerOut ) const
{
	if ( mPointWorldPositions.size() > 0 )
	{
		*pFirstCornerOut = mPointWorldPositions[ 0 ];
		*pSecondCornerOut = mPointWorldPositions[ 0 ];

		for ( uint32_t pointIdx = 1; pointIdx < mPointWorldPositions.size(); pointIdx++ )
		{
			const Eigen::Vector3f& p = mPointWorldPositions[ pointIdx ];

			if ( p[ 0 ] < (*pFirstCornerOut)[ 0 ] ) (*pFirstCornerOut)[ 0 ] = p[ 0 ];
			if ( p[ 1 ] < (*pFirstCornerOut)[ 1 ] ) (*pFirstCornerOut)[ 1 ] = p[ 1 ];
			if ( p[ 2 ] < (*pFirstCornerOut)[ 2 ] ) (*pFirstCornerOut)[ 2 ] = p[ 2 ];

			if ( p[ 0 ] > (*pSecondCornerOut)[ 0 ] ) (*pSecondCornerOut)[ 0 ] = p[ 0 ];
			if ( p[ 1 ] > (*pSecondCornerOut)[ 1 ] ) (*pSecondCornerOut)[ 1 ] = p[ 1 ];
			if ( p[ 2 ] > (*pSecondCornerOut)[ 2 ] ) (*pSecondCornerOut)[ 2 ] = p[ 2 ];
		}
	}
	else
	{
		// No points so return default bounding box
		*pFirstCornerOut = Eigen::Vector3f( -1.0f, -1.0f, -1.0f );
		*pSecondCornerOut = Eigen::Vector3f( 1.0f, 1.0f, 1.0f );
	}
}

//--------------------------------------------------------------------------------------------------
float PointCloud::pickSurface( const Eigen::Vector3f& lineStart, const Eigen::Vector3f& lineDir,
    int32_t* pClosestPointIdxOut, float close ) const
{
    float dirLength = lineDir.norm();

    if ( dirLength <= 0.0 )
    {
        return -1.0;
    }

    Eigen::Vector3f normLineDir = lineDir / dirLength;

    bool bPointFound = false;
    float closestRayToPointSquared = FLT_MAX;
    float distanceAlongRayToClosestApproach = -1.0;
   int32_t closestPointIdx = INVALID_POINT_IDX;

	//printf( "Start Pos is %f %f %f\n", lineStart[ 0 ], lineStart[ 1 ], lineStart[ 2 ] );
    //printf( "Dir length is %f\n", normLineDir.squaredNorm() );

    uint32_t pointIdx = 0;
    for  ( ; pointIdx < mPointWorldPositions.size(); pointIdx++ )
    {
        const Eigen::Vector3f& pos = mPointWorldPositions[ pointIdx ];

        Eigen::Vector3f vectorToPos = pos - lineStart;
        float distanceToClosestApproach = vectorToPos.dot( normLineDir );

        if ( distanceToClosestApproach > 0.0f )
        {
            Eigen::Vector3f closestPoint = lineStart + distanceToClosestApproach*normLineDir;

            float rayToPointSquared = ( pos - closestPoint ).squaredNorm();

            if ( rayToPointSquared < closestRayToPointSquared )
            {
                closestRayToPointSquared = rayToPointSquared;

                if ( rayToPointSquared < close*close )
                {
                    // The ray comes close enough for contact
                    bPointFound = true;
                    closestPointIdx = pointIdx;
                    distanceAlongRayToClosestApproach = distanceToClosestApproach;
                }
            }
        }
    }

    //printf( "Checked %u points\n", pointIdx );

    //printf( "Closest we got was %f\n", sqrtf( closestRayToPointSquared ) );

    if ( !bPointFound )
    {
        return -1.0f;
    }
    else
    {
        if ( NULL != pClosestPointIdxOut )
        {
            *pClosestPointIdxOut = closestPointIdx;
        }

        return distanceAlongRayToClosestApproach;
    }
}
