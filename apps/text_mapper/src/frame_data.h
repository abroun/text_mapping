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
// File: frame_data.h
// Desc: A simple struct for holding the properties and data of a frame
//--------------------------------------------------------------------------------------------------

#ifndef FRAME_DATA_H_
#define FRAME_DATA_H_

//--------------------------------------------------------------------------------------------------
#include <string>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include "text_mapping/box_filter.h"
#include "text_mapping/point_cloud.h"

//--------------------------------------------------------------------------------------------------
struct FrameData
{
    public: FrameData()
    {
        mModelInFrameSpaceTransform = Eigen::Matrix4f::Identity();
        mbModelInFrameSpaceTransformSet = false;
        mbHasBoxFilter = false;
    }

    std::string mHighResImageFilename;
	std::string mKinectColorImageFilename;
	std::string mKinectDepthPointCloudFilename;

    cv::Mat mHighResImage;
    cv::Mat mKinectColorImage;
    PointCloud::Ptr mpKinectDepthPointCloud;

    private: Eigen::Matrix4f mModelInFrameSpaceTransform;
    private: bool mbModelInFrameSpaceTransformSet;

    private: BoxFilter mBoxFilter;
    private: bool mbHasBoxFilter;

    //! Tries to load in the images referenced by the filenames.
    //! @param bShowErrorMsgBox If set to true, the user will be shown a message box telling them
    //!        about the missing image
    //! @return true if all the images were loaded, and false otherwise
    public: bool tryToLoadImages( bool bShowErrorMsgBox );

    //! Unloads images to save memory
    public: void unloadImages();

    public: void setModelInFrameSpaceTransform( const Eigen::Matrix4f& transform )
    {
        mModelInFrameSpaceTransform = transform;
        mbModelInFrameSpaceTransformSet = true;
    }

    public: const Eigen::Matrix4f& getModelInFrameSpaceTransform() const
    {
        return mModelInFrameSpaceTransform;
    }

    public: bool isModelInFrameSpaceTransformSet() const { return mbModelInFrameSpaceTransformSet; }

    public: void setBoxFilter( const BoxFilter& boxFilter )
    {
        mBoxFilter = boxFilter;
        mbHasBoxFilter = true;
    }

    public: const BoxFilter& getBoxFilter() const
    {
        return mBoxFilter;
    }

    public: bool hasBoxFilter() const { return mbHasBoxFilter; }

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // FRAME_DATA_H_
