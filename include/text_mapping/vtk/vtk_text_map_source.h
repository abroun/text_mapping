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

#ifndef VTK_TEXT_MAP_SOURCE_H_
#define VTK_TEXT_MAP_SOURCE_H_

//--------------------------------------------------------------------------------------------------
#include <vtkPolyDataAlgorithm.h>
#include "text_mapping/text_map.h"

//--------------------------------------------------------------------------------------------------
//! Produces the PolyData used to display a TextMap with VTK.
class vtkTextMapSource : public vtkPolyDataAlgorithm
{
    public: vtkTypeMacro( vtkTextMapSource, vtkPolyDataAlgorithm );
    public: void PrintSelf( std::ostream& os, vtkIndent indent );

    // Constructs an empty TextMap source
    public: static vtkTextMapSource* New();

    // Sets the surfel model for the class
    public: vtkSetMacro( TextMapPtr, TextMap::ConstPtr );
    public: vtkGetMacro( TextMapPtr, TextMap::ConstPtr );
    
    public: void SetFrameTransform( const Eigen::Matrix4f& transform )
    {
    	mFrameTransform = transform;
    	this->Modified();
    }

    protected: vtkTextMapSource();
    protected: ~vtkTextMapSource() {}

    protected: virtual int RequestData( vtkInformation *, vtkInformationVector **, vtkInformationVector * );
    protected: virtual int RequestInformation( vtkInformation *, vtkInformationVector **, vtkInformationVector * );

    protected: TextMap::ConstPtr TextMapPtr;
    Eigen::Matrix4f mFrameTransform;
    
    private: vtkTextMapSource( const vtkTextMapSource& );  // Not implemented.
    void operator=( const vtkTextMapSource& );  // Not implemented.

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // VTK_TEXT_MAP_SOURCE_H_
