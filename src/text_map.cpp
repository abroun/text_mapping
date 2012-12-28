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
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Geometry>
#include "text_mapping/text_map.h"
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
// TextMap
//--------------------------------------------------------------------------------------------------
const std::string TextMap::NO_MODEL_FILENAME( "NO_MODEL" );

//--------------------------------------------------------------------------------------------------
TextMap::TextMap()
    : mModelFilename( NO_MODEL_FILENAME )
{
}

//--------------------------------------------------------------------------------------------------
TextMap::Ptr TextMap::loadTextMapFromFile( const std::string& filename )
{
    TextMap::Ptr pTextMap;
    
    std::ifstream mapFile( filename );
    if ( mapFile.is_open() )
    {
        pTextMap = TextMap::Ptr( new TextMap() );
        
        std::string curLine;
        
        // First read out the object filename
        if ( mapFile.good() )
        {
            std::getline( mapFile, curLine );
            boost::trim( curLine );
            
            if ( curLine != NO_MODEL_FILENAME )
            {                
                pTextMap->mModelFilename = Utilities::decodeRelativeFilename( filename, curLine );
                
                printf( "Got a model filename of %s\n", pTextMap->mModelFilename.c_str() );
            }
            
            // Now read in the number of letters
            uint32_t numLetters;
            mapFile >> numLetters;
            
            pTextMap->mLetters.reserve( numLetters );
            
            // Read in the letters
            for ( uint32_t letterIdx = 0; letterIdx < numLetters; letterIdx++ )
            {
                Letter letter;
                Eigen::Vector3f pos;
                Eigen::Vector3f eulerAnglesDegrees;
                
                mapFile >> letter.mCharacter;
                mapFile >> pos[ 0 ] >> pos[ 1 ] >> pos[ 2 ];
                mapFile >> eulerAnglesDegrees[ 0 ] >> eulerAnglesDegrees[ 1 ] >> eulerAnglesDegrees[ 2 ];
                mapFile >> letter.mWidth >> letter.mHeight;
                
                letter.mMtx = Eigen::Matrix4f::Identity();
                letter.mMtx.block<3,3>( 0, 0 ) = 
                    ( Eigen::AngleAxisf( Utilities::degToRad( eulerAnglesDegrees[ 0 ] ), Eigen::Vector3f::UnitX() )
                    * Eigen::AngleAxisf( Utilities::degToRad( eulerAnglesDegrees[ 1 ] ), Eigen::Vector3f::UnitY() )
                    * Eigen::AngleAxisf( Utilities::degToRad( eulerAnglesDegrees[ 2 ] ), Eigen::Vector3f::UnitZ() ) ).toRotationMatrix();
                letter.mMtx.block<3,1>( 0, 3 ) = pos;
            
                pTextMap->mLetters.push_back( letter );
            }
        }
        
        mapFile.close();
    }
    
    return pTextMap;
}
    
//--------------------------------------------------------------------------------------------------
void TextMap::saveToFile( const std::string& filename )
{
    FILE* pMapFile = fopen( filename.c_str(), "w" );
    if ( NULL != pMapFile )
    {
        std::string relativeModelFilename; 
        if ( NO_MODEL_FILENAME == mModelFilename )
        {
            relativeModelFilename = NO_MODEL_FILENAME;
        }
        else
        {
            relativeModelFilename = Utilities::createRelativeFilename( filename, mModelFilename );
        }
        
        // Write out the path to the model
        fprintf( pMapFile, "%s\n", relativeModelFilename.c_str() );
        
        // Write out the number of letters in the map
        fprintf( pMapFile, "%u\n", (uint32_t)mLetters.size() );
        
        // Write out each letter
        for ( uint32_t letterIdx = 0; letterIdx < mLetters.size(); letterIdx++ )
        {
            const Letter& letter = mLetters[ letterIdx ];

            Eigen::Vector3f eulerAngles = letter.mMtx.block<3,3>( 0, 0 ).eulerAngles( 0, 1, 2 );
            const Eigen::Vector3f& pos = letter.mMtx.block<3,1>( 0, 3 );
            
            fprintf( pMapFile, "%c %f %f %f %f %f %f %f %f\n", 
                letter.mCharacter, pos[ 0 ], pos[ 1 ], pos[ 2 ],
                Utilities::radToDeg( eulerAngles[ 0 ] ), 
                Utilities::radToDeg( eulerAngles[ 1 ] ), 
                Utilities::radToDeg( eulerAngles[ 2 ] ), letter.mWidth, letter.mHeight );
        }
        
        fclose( pMapFile );
    }
}

//--------------------------------------------------------------------------------------------------
void TextMap::getBoundingBox( Eigen::Vector3f* pFirstCornerOut, Eigen::Vector3f* pSecondCornerOut ) const
{
    // TODO: Implement this properly
    *pFirstCornerOut = -Eigen::Vector3f( 1.0, 1.0, 1.0 );
    *pSecondCornerOut = Eigen::Vector3f( 1.0, 1.0, 1.0 );
}
   
