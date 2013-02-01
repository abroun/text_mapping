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
// File: text_detection.h
// Desc: Header file for main text_detection routines
//--------------------------------------------------------------------------------------------------

#ifndef TEXT_DETECTION_H
#define TEXT_DETECTION_H

//--------------------------------------------------------------------------------------------------
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>

//--------------------------------------------------------------------------------------------------
struct Letter2D
{
    Letter2D() {}
    Letter2D( char c, double tl_x, double tl_y, double tr_x, double tr_y,
        double bl_x, double bl_y, double br_x, double br_y )
        : mCharacter( c ),
        mTopLeft( tl_x, tl_y ),
        mTopRight( tr_x, tr_y ),
        mBottomLeft( bl_x, bl_y ),
        mBottomRight( br_x, br_y ) {}

    char mCharacter;
    Eigen::Vector2d mTopLeft;
    Eigen::Vector2d mTopRight;
    Eigen::Vector2d mBottomLeft;
    Eigen::Vector2d mBottomRight;

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION( Letter2D );

typedef std::vector<Letter2D, Eigen::aligned_allocator<Letter2D> > Letter2DVector;

//--------------------------------------------------------------------------------------------------
Letter2DVector detect_text(cv::Mat inputImage, uint32_t frameIdx );

#endif
