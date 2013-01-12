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

#ifndef BOX_FILTER_H_
#define BOX_FILTER_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <vector>
#include <Eigen/Core>

//--------------------------------------------------------------------------------------------------
//! Defines a 3D box that can be used to filter point clouds
class BoxFilter
{
    public: BoxFilter()
        : mTransform( Eigen::Matrix4f::Identity() ), mDimensions( 1.0f, 1.0f, 1.0f ) {}
    public: BoxFilter( const Eigen::Matrix4f& transform, const Eigen::Vector3f& dimensions )
        : mTransform( transform ), mDimensions( dimensions ) {}

    //! Calculates the 8 corners of the bounding box, and returns them in a vector
    //! @return A vector of corners in the order
    //!             top-front-left
    //!             top-back-left
    //!             top-back-right
    //!             top-front-right
    //!             bottom-front-left
    //!             bottom-back-left
    //!             bottom-back-right
    //!             bottom-front-right
    public: std::vector<Eigen::Vector3f> calculateCorners() const;

    public: Eigen::Matrix4f mTransform;
    public: Eigen::Vector3f mDimensions;

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


#endif // BOX_FILTER_H_
