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

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

//--------------------------------------------------------------------------------------------------
//! Holds a point cloud from a depth camera, such as the Kinect.
class PointCloud
{
	public: typedef boost::shared_ptr<PointCloud> Ptr;
    public: typedef boost::shared_ptr<const PointCloud> ConstPtr;

    //! Index returned due to invalid depth
    public: static const int32_t INVALID_POINT_IDX = -1;

    //! The default constructor for a PointCloud
    public: PointCloud();

    //! Constructor for the PointCloud
	public: PointCloud( uint32_t width, uint32_t height, float focalLengthMM );

    //! The destructor
    public: virtual ~PointCloud();

    //! Loads a PointCloud from a Simple Point Cloud (SPC) file.
    //! @param filename The name of the file to read the PointCloud from
    public: static PointCloud::Ptr loadTextMapFromSpcFile( const std::string& filename );

    //! Gets the focal length (in mm) of the depth camera used to capture the point cloud
    public: float getFocalLengthMM() const { return mFocalLengthMM; }

    //! Gets the color image associated with the point cloud
    public: const cv::Mat& getImage() const { return mImage; }

    //! Returns the number of points in the point cloud
    public: uint32_t getNumPoints() const { return mPointWorldPositions.size(); }

    //! Gets the index of the point that corresponds to coordinates on the image plane.
    //! @return The index of the point at the position, or INVALID_POINT_IDX if there is no point
    public: int32_t getPointIdxAtImagePos( int32_t u, int32_t v ) const;

    //! Gets the position of a point in the camera space of the depth camera which captured the image
	//! @return The position of the point
    public: Eigen::Vector3f getPointWorldPos( int32_t pointIdx ) const { return mPointWorldPositions[ pointIdx ]; }

    //! Gets the position of a point in the image space of the depth camera which captured the image
	//! @return The 2D image position of the point
	public: Eigen::Vector2f getPointImagePos( int32_t pointIdx ) const;

    private: float mFocalLengthMM;
    private: cv::Mat mImage;
    private: std::vector<int32_t> mPointMap;	//! A 2D map containing point indices
    private: std::vector<Eigen::Vector3f> mPointWorldPositions;
};

#endif // POINT_CLOUD_H_
