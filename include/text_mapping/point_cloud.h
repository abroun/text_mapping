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
#include "text_mapping/box_filter.h"

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
    //! @param width The width of the depth camera image plane in pixels
    //! @param height The height of the depth camera image plane in pixels
    //! @param focalLengthPixels The focal length of the depth camera in pixels
	public: PointCloud( uint32_t width, uint32_t height, float focalLengthPixels );

    //! The destructor
    public: virtual ~PointCloud();

    //! Loads a PointCloud from a Simple Point Cloud (SPC) file.
    //! @param filename The name of the file to read the PointCloud from
    public: static PointCloud::Ptr loadPointCloudFromSpcFile( const std::string& filename );

    //! Saves a PointCloud to a Simple Point Cloud (SPC) file.
    //! @param filename The name of the file to save the PointCloud to.
    //! @param bBinary If true, the binary format is used. Otherwise the ascii format is used.
    public: void saveToSpcFile( const std::string& filename, bool bBinary );

    //! Creates and returns a filtered version of the point cloud, where each point is only allowed
    //! through if it is within a certain distance of one or more of a set of filter points
    //! @param filterPoints A list of points to test against
    //! @param filterDistance If a point is not within this distance of at least one of the filter
    //!        points it is filtered out.
    //! @return A new point cloud containing all of the points that weren't filtered out
    public: PointCloud::Ptr filterOutPointsFarFromPointSet(
		const std::vector<Eigen::Vector3f>& filterPoints, float filterDistance ) const;

    //! Creates and returns a filtered version of the point cloud, where each point is only allowed
    //! through if it is within a box filter
    //! @param boxFilter The box to filter the points against
    //! @return A new point cloud containing all of the points that weren't filtered out
    public: PointCloud::Ptr filterWithBoxFilter( const BoxFilter& boxFilter ) const;

    //! Gets the focal length of the depth camera used to capture the point cloud
    public: float getFocalLengthInPixels() const { return mFocalLengthPixels; }

    //! Gets the color image associated with the point cloud
    public: const cv::Mat& getImage() const { return mImage; }

    //! Returns the number of points in the point cloud
    //! @return Number of points in the point cloud
    public: uint32_t getNumPoints() const { return mPointWorldPositions.size(); }

    //! Gets the index of the point that corresponds to coordinates on the image plane.
    //! @param u The horizontal coordinate of the image position
	//! @param v The vertical coordinate of the image position
    //! @return The index of the point at the position, or INVALID_POINT_IDX if there is no point
    public: int32_t getPointIdxAtImagePos( int32_t u, int32_t v ) const;

    //! Gets the position of a point in the camera space of the depth camera which captured the image
    //! @param pointIdx The index of the desired point
	//! @return The position of the point
    public: Eigen::Vector3f getPointWorldPos( int32_t pointIdx ) const { return mPointWorldPositions[ pointIdx ]; }

    //! Gets the position of a point in the image space of the depth camera which captured the image
    //! @param pointIdx The index of the desired point
	//! @return The 2D image position of the point
	public: Eigen::Vector2f getPointImagePos( int32_t pointIdx ) const;

	//! Projects a position in the point cloud world space onto the image plane of the point cloud
	//! @param worldPos The position in world space
	//! @return The 2D image position of the position
	public: Eigen::Vector2f convertWorldPosToImagePos( const Eigen::Vector3f& worldPos ) const;

	//! Gets the color of a point based on its position in the image space of the depth camera
	//! which captured the image
	//! @param pointIdx The index of the desired point
	//! @param[out] pRedOut Variable to hold the red channel
	//! @param[out] pGreenOut Variable to hold the green channel
	//! @param[out] pBlueOut Variable to hold the blue channel
	//! @param[out] pAlphaOut Variable to hold the alpha channel
	public: void getPointColor( int32_t pointIdx,
		uint8_t* pRedOut, uint8_t* pGreenOut, uint8_t* pBlueOut, uint8_t* pAlphaOut ) const;

	//! Adds a new point to the point cloud at a given world position. If the new point would
	//! project onto the same screen position as an existing point then it will replace it. If the
	//! new point does not actually project into the image then it won't be added.
	//! @param worldPos The position of the new point
	//! @param r The red component of the points colour
	//! @param g The blue component of the points colour
	//! @param b The green component of the points colour
	//! @param a The alpha component of the points colour
	//! @return The index of the new point. If the point wasn't added then we return INVALID_POINT_IDX
	public: int32_t addPoint( const Eigen::Vector3f& worldPos, uint8_t r, uint8_t g, uint8_t b, uint8_t a );

	//! Returns the 3D extents of the PointCloud
	//! @param[out] pFirstCornerOut Variable to hold the first bounding box corner
	//! @param[out] pSecondCornerOut Variable to hold the second bounding box corner
	public: void getBoundingBox( Eigen::Vector3f* pFirstCornerOut, Eigen::Vector3f* pSecondCornerOut ) const;

	//! Casts a ray into the point cloud. If the ray gets close enough to a point then we assume
	//! that we've hit a surface and return the distance to the surface.
	//! @param lineStart The start of the ray
	//! @param lineDir The direction of the ray
	//! @param pClosestPointIdxOut Optional pointer that can be used to find the point the ray 'hit'
	//! @param close How close the ray has to pass to a point for it to be considered a hit
	//! @return The distance along the ray to get to the 'surface', or -1 if we didn't come close
	//!         to any point.
    public: float pickSurface( const Eigen::Vector3f& lineStart, const Eigen::Vector3f& lineDir,
         int32_t* pClosestPointIdxOut=NULL, float close=0.002f ) const;

    //! A helper routine for copying points when implementing filters
    //! @param pointIdx The index of the point to copy
    //! @param pPointCloud The target point cloud
    protected: void copyPointToPointCloud( int32_t pointIdx, Ptr pPointCloud ) const;

    private: float mFocalLengthPixels;
    private: cv::Mat mImage;
    private: std::vector<int32_t> mPointMap;	//! A 2D map containing point indices
    private: std::vector<Eigen::Vector3f> mPointWorldPositions;
};

#endif // POINT_CLOUD_H_
