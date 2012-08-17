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

#ifndef TEXT_MAPPING_H
#define TEXT_MAPPING_H

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "text_mapping/letter.h"

//--------------------------------------------------------------------------------------------------
//! \brief Data structure to hold a map of text on the surface of a 3D object. 
//! Text is stored as letters with each letter having a position, orientation and bounding box.
class TextMap
{
    public: typedef boost::shared_ptr<TextMap> Ptr;
    public: typedef boost::shared_ptr<const TextMap> ConstPtr;

    //! Placeholder used when no model filename is available
    public: static const std::string NO_MODEL_FILENAME;
    
    //! The default constructor for a TextMap
    public: TextMap();
    
    //! Loads a TextMap from a file.
    //! @param filename The name of the file to read the TextMap from
    public: static TextMap::Ptr loadTextMapFromFile( const std::string& filename );
    
    //! Saves the contents of the TextMap to a file.
    //! @param filename The name of the file to write the TextMap to.
    public: void saveToFile( const std::string& filename );
   
    //! Gets the number of letters in the map.
    public: size_t getNumLetters() const { return mLetters.size(); }
    
    //! Gets the model filename. Returns NO_MODEL_FILENAME is no model is attached
    public: const std::string& getModelFilename() const { return mModelFilename; }
    
    //! Sets the model filename for the TextMap.
    //! @param The filename of a model of the object that the TextMap maps
    public: void setModelFilename( const std::string& modelFilename ) { mModelFilename = modelFilename; }
    
    //! A const accessor for individual letters
    //! @param letterIdx The index of the letter to access
    public: const Letter& getLetter( uint32_t letterIdx ) const { return mLetters[ letterIdx ]; }
    
    //! A non const accessor for individual letters
    //! @param letterIdx The index of the letter to access
    public: Letter& getLetter( uint32_t letterIdx ) { return mLetters[ letterIdx ]; }
    
    //! Returns the 3D extents of the TextMap
    //! @param[out] pFirstCornerOut Variable to hold the first bounding box corner
    //! @param[out] pSecondCornerOut Variable to hold the second bounding box corner
    public: void getBoundingBox( Eigen::Vector3f* pFirstCornerOut, Eigen::Vector3f* pSecondCornerOut ) const;
    
    //! Add a new letter to the TextMap
    //! @param letter The letter to add to the TextMap
    public: void addLetter( const Letter& letter ) { mLetters.push_back( letter ); } 
    
    //! Deletes the letter at the given index from the TextMap
    //! @param letterIdx The index of the letter to delete
    public: void deleteLetter( uint32_t letterIdx ) { mLetters.erase( mLetters.begin() + letterIdx ); }
    
    private: std::string mModelFilename;    // Optional filename of a model of the object the map is on
    private: std::vector<Letter, Eigen::aligned_allocator<Letter> > mLetters;
    
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // TEXT_MAPPING_H
