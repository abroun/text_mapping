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
// File: key_point.cpp
// Desc: Represents a key point used to align frames with each other, and with the object model
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include "key_point.h"

//--------------------------------------------------------------------------------------------------
// KeyPoint
//--------------------------------------------------------------------------------------------------
KeyPoint::KeyPoint()
    : mbHasModelInstance( false)
{

}

//--------------------------------------------------------------------------------------------------
KeyPoint::~KeyPoint()
{

}

//--------------------------------------------------------------------------------------------------
void KeyPoint::addKeyPointFrameInstance( int32_t frameIdx, const Eigen::Vector3f& instancePos )
{
    KeyPointInstance frameInstance;
    frameInstance.mPos = instancePos;

    mFrameInstanceMap[ frameIdx ] = frameInstance;
}

//--------------------------------------------------------------------------------------------------
void KeyPoint::removeKeyPointFrameInstance( int32_t frameIdx )
{
    InstanceMap::iterator mapIter = mFrameInstanceMap.find( frameIdx );

    if ( mFrameInstanceMap.end() != mapIter )
    {
        mFrameInstanceMap.erase( mapIter );
    }
}

//--------------------------------------------------------------------------------------------------
std::vector<int32_t> KeyPoint::getKeyPointFrameInstanceIndices() const
{
    std::vector<int32_t> indices;
    indices.reserve( mFrameInstanceMap.size() );

    for ( InstanceMap::const_iterator mapIter = mFrameInstanceMap.begin();
        mapIter != mFrameInstanceMap.end(); mapIter++ )
    {
        indices.push_back( mapIter->first );
    }

    return indices;
}

//--------------------------------------------------------------------------------------------------
const KeyPointInstance* KeyPoint::getKeyPointFrameInstance( int32_t frameIdx ) const
{
    InstanceMap::const_iterator mapIter = mFrameInstanceMap.find( frameIdx );

    if ( mFrameInstanceMap.end() != mapIter )
    {
        return &(mapIter->second);
    }
    else
    {
        return NULL;
    }
}

//--------------------------------------------------------------------------------------------------
void KeyPoint::addKeyPointModelInstance( const Eigen::Vector3f& instancePos )
{
    mModelInstance.mPos = instancePos;
    mbHasModelInstance = true;
}

//--------------------------------------------------------------------------------------------------
void KeyPoint::removeKeyPointModelInstance()
{
    mbHasModelInstance = false;
}



