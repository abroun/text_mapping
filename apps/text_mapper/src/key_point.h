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
// File: key_point.h
// Desc: Represents a key point used to align frames with each other, and with the object model
//--------------------------------------------------------------------------------------------------

#ifndef KEY_POINT_H_
#define KEY_POINT_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <map>
#include <vector>
#include <Eigen/Core>

//--------------------------------------------------------------------------------------------------
//! A 3D instance of a key point in a frame, or on the surface of a model
struct KeyPointInstance
{
    Eigen::Vector3f mPos;
};

//--------------------------------------------------------------------------------------------------
//! Struct for carrying around a key point, and non permanent data, such as display colour, size etc
struct KeyPointInstanceData
{
	KeyPointInstanceData() : mbDisplayAsCross( false ) {}
	KeyPointInstanceData( const KeyPointInstance& keyPointInstance,
		uint8_t r, uint8_t g, uint8_t b, bool bDisplayAsCross=false )
		: mKeyPointInstance( keyPointInstance ),
		  mbDisplayAsCross( bDisplayAsCross ),
		  mR( r ), mG( g ), mB( b ) {}

	KeyPointInstance mKeyPointInstance;
	Eigen::Vector2f mProjectedPosition;
	bool mbDisplayAsCross;
	uint8_t mR;
	uint8_t mG;
	uint8_t mB;

	public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

//--------------------------------------------------------------------------------------------------
class KeyPoint
{
    public: KeyPoint();
    public: virtual ~KeyPoint();

    public: void addKeyPointFrameInstance( int32_t frameIdx, const Eigen::Vector3f& instancePos );
    public: void removeKeyPointFrameInstance( int32_t frameIdx );
    public: std::vector<int32_t> getKeyPointFrameInstanceIndices() const;
    public: const KeyPointInstance* getKeyPointFrameInstance( int32_t frameIdx ) const;

    public: void addKeyPointModelInstance( const Eigen::Vector3f& instancePos );
    public: void removeKeyPointModelInstance();
    public: bool hasKeyPointModelInstanceIndices() const { return mbHasModelInstance; }
    public: const KeyPointInstance* getKeyPointModelInstance() const
    {
        return mbHasModelInstance ? &mModelInstance : NULL;
    }

    private: typedef std::map<int32_t, KeyPointInstance> InstanceMap;
    private: InstanceMap mFrameInstanceMap;

    private: KeyPointInstance mModelInstance;
    private: bool mbHasModelInstance;
};


#endif // KEY_POINT_H_
