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
#include <cstdlib>
#include <boost/filesystem.hpp>
#include "text_mapping/utilities.h"

//--------------------------------------------------------------------------------------------------
// Utilities
//--------------------------------------------------------------------------------------------------
std::string Utilities::createRelativeFilename( const std::string& baseFilename, 
                                               const std::string& otherFilename )
{
    std::string relativeFilename;
    
    boost::filesystem::path baseFilenamePath( baseFilename );
    boost::filesystem::path otherFilenamePath( otherFilename );
    
    // Get iterators to the beginning and end of each path
    boost::filesystem::path::iterator lastBaseFilenamePathElement = --baseFilenamePath.parent_path().end();
    boost::filesystem::path::iterator lastOtherFilenamePathElement = --otherFilenamePath.parent_path().end();
    
    boost::filesystem::path::iterator curBaseFilenamePathElement = baseFilenamePath.begin();
    boost::filesystem::path::iterator curOtherFilenamePathElement = otherFilenamePath.begin();
    
    // Step through all common elements of the two paths
    bool bFilesInSameDirectory = false;
    while ( true )
    {
        if ( *curBaseFilenamePathElement == *curOtherFilenamePathElement )
        {
            if ( *curBaseFilenamePathElement == *lastBaseFilenamePathElement )
            {
                bFilesInSameDirectory = true;
                break;
            }
            
            else
            {
                curBaseFilenamePathElement++;
                curOtherFilenamePathElement++;
            }
        }
        else
        {
            // We've found a point where the paths diverge
            break;
        }
    }
    
    if ( bFilesInSameDirectory )
    {
        relativeFilename = otherFilenamePath.filename().string();
    }
    else
    {        
        // Compose the relative path with ../ elements for each remaining element of the filename path
        int numRemainingElements = 1;
        while ( *curBaseFilenamePathElement != *lastBaseFilenamePathElement )
        {
            curBaseFilenamePathElement++;
            numRemainingElements++;
        }
        
        for ( int i = 0; i < numRemainingElements; i++ )
        {
            relativeFilename += "../";
        }

        // Then append on the remaining elements of the other filename path
        while ( *curOtherFilenamePathElement != otherFilenamePath.filename() )
        {
            relativeFilename += (*curOtherFilenamePathElement).string();
            relativeFilename += "/";
            
            curOtherFilenamePathElement++;
        }
        relativeFilename += otherFilenamePath.filename().string();
    }
    
    return relativeFilename;
}

//--------------------------------------------------------------------------------------------------
std::string Utilities::decodeRelativeFilename( const std::string& baseFilename,
                                               const std::string& relativeFilename )
{
    // Check that the relative filename is actually relative before preceding
    boost::filesystem::path relativeFilenamePath( relativeFilename );
    if ( relativeFilenamePath.is_absolute() )
    {
        return relativeFilename;
    }

    boost::filesystem::path basePath = boost::filesystem::path( baseFilename ).parent_path();

    boost::filesystem::path::iterator basePathEndIter = basePath.end();

    std::string otherFilename = relativeFilename;
    while ( otherFilename.find( "../" ) == 0 )
    {
        otherFilename = otherFilename.substr( 3 );
        basePathEndIter--;
    }

    boost::filesystem::path filePath;
    for ( boost::filesystem::path::iterator basePathIter = basePath.begin();
        basePathIter != basePathEndIter; basePathIter++ )
    {
        filePath /= *basePathIter;
    }
    filePath /= otherFilename;

    return filePath.string();
}

//--------------------------------------------------------------------------------------------------
std::string Utilities::makeFilenameAbsoluteFromCWD( const std::string& filename )
{
    boost::filesystem::path filenamePath( filename );
    std::string absFilename;
    if ( filenamePath.is_absolute() )
    {
        absFilename = filename;
    }
    else
    {
        absFilename = boost::filesystem::absolute(
            filenamePath, boost::filesystem::current_path() ).string();
    }

    return absFilename;
}

//--------------------------------------------------------------------------------------------------
std::string Utilities::getDataDir()
{
    const char* pDirectoryName = std::getenv( "TEXT_MAPPING_DATA_DIR" );

    std::string directoryName;
    if ( NULL == pDirectoryName )
    {
        directoryName = "../data";
    }
    else
    {
        directoryName = std::string( pDirectoryName );
    }

     boost::filesystem::path directoryPath( directoryName );
     return boost::filesystem::absolute( directoryPath ).string();
}
