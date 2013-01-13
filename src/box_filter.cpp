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
#include "text_mapping/box_filter.h"
#include <opencv2/core/eigen.hpp>

//--------------------------------------------------------------------------------------------------
// BoxFilter
//--------------------------------------------------------------------------------------------------
bool BoxFilter::readBoxFilterFromFileStorage( cv::FileNode& fileNode, BoxFilter* pBoxFilterOut )
{
    bool bReadSuccessful = false;

    cv::FileNode transformNode = fileNode[ "Transform" ];

    if ( !transformNode.empty() )
    {
        cv::Mat transform;
        transformNode >> transform;
        cv2eigen( transform, pBoxFilterOut->mTransform );

        cv::FileNode dimensionsNode = fileNode[ "Dimensions" ];
        if ( !dimensionsNode.empty() )
        {
            cv::Mat dimensions;
            dimensionsNode >> dimensions;
            cv2eigen( dimensions, pBoxFilterOut->mDimensions );

            bReadSuccessful = true;
        }
    }

    return bReadSuccessful;
}

//--------------------------------------------------------------------------------------------------
void BoxFilter::writeToFileStorage( cv::FileStorage& fileStorage, const std::string& nodeName ) const
{
    fileStorage << nodeName << "{";

    cv::Mat transform;
    eigen2cv( mTransform, transform );
    fileStorage << "Transform" << transform;

    cv::Mat dimensions;
    eigen2cv( mDimensions, dimensions );
    fileStorage << "Dimensions" << dimensions;

    fileStorage << "}";
}

//--------------------------------------------------------------------------------------------------
std::vector<Eigen::Vector3f> BoxFilter::calculateCorners() const
{
    std::vector<Eigen::Vector3f> corners;
    corners.reserve( 8 );

    Eigen::Vector3f boxCentre = mTransform.block<3,1>( 0, 3 );
    Eigen::Vector3f halfDimensions = mDimensions/2.0f;
    Eigen::Vector3f halfX = halfDimensions[ 0 ]*mTransform.block<3,1>( 0, 0 );
    Eigen::Vector3f halfY = halfDimensions[ 1 ]*mTransform.block<3,1>( 0, 1 );
    Eigen::Vector3f halfZ = halfDimensions[ 2 ]*mTransform.block<3,1>( 0, 2 );

    corners.push_back( Eigen::Vector3f( boxCentre + halfY - halfZ + halfX ) ); // top-front-left
    corners.push_back( Eigen::Vector3f( boxCentre + halfY + halfZ + halfX ) ); // top-back-left
    corners.push_back( Eigen::Vector3f( boxCentre + halfY + halfZ - halfX ) ); // top-back-right
    corners.push_back( Eigen::Vector3f( boxCentre + halfY - halfZ - halfX ) ); // top-front-right
    corners.push_back( Eigen::Vector3f( boxCentre - halfY - halfZ + halfX ) ); // bottom-front-left
    corners.push_back( Eigen::Vector3f( boxCentre - halfY + halfZ + halfX ) ); // bottom-back-left
    corners.push_back( Eigen::Vector3f( boxCentre - halfY + halfZ - halfX ) ); // bottom-back-right
    corners.push_back( Eigen::Vector3f( boxCentre - halfY - halfZ - halfX ) ); // bottom-front-right

    return corners;
}
