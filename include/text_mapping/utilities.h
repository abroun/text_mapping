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

#ifndef UTILITIES_H
#define UTILITIES_H

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <Eigen/Core>

//--------------------------------------------------------------------------------------------------
//! \brief Dumping ground for utility routines that have yet to find a proper home elsewhere. 
class Utilities
{
    //! Converts degrees into radians
    //! @param degrees The input angle in degrees
    //! @return The angle converted to radians
    public: static float degToRad( float degrees ) { return (float)(degrees*M_PI/180.0); }

    //! Converts degrees into radians
    //! @param degrees The input angle in degrees
    //! @return The angle converted to radians
    public: static double degToRad( double degrees ) { return degrees*M_PI/180.0; }

    //! Converts radians into degrees
    //! @param radians The input angle in radians
    //! @return The angle converted to degrees
    public: static float radToDeg( float radians ) { return (float)(radians*180.0/M_PI); }
    
    //! Takes two absolute filenames, and returns a relative filename for the second filename
    //! from the directory of the first filename.
    //! TODO: This needs a lot of unit testing...
    //! @param baseFilename The first absolute filename
    //! @param otherFilename The second absolute filename
    //! @return A relative filename for the otherFilename which goes from directory of the 
    //!         baseFilename
    public: static std::string createRelativeFilename( const std::string& baseFilename, 
        const std::string& otherFilename );

    //! Given a filename relative to another filename. This routine returns the combined filename
    //! TODO: This needs a lot of unit testing...
    //! @param baseFilename The first filename. The relative filename is relative to this file
    //! @param relativeFilename The relative filename
    //! @return A combined filename
    public: static std::string decodeRelativeFilename( const std::string& baseFilename,
        const std::string& relativeFilename );

    //! Makes sure that a filename is absolute
    //! @param filename The filename to check, and if needed, to modify
    //! @return If filename is absolute, then it will be returned unmodified. Otherwise
    //!         it will be assumed that the path is relative to the current working directory
    //!         and the corresponding absolute path will be returned.
    public: static std::string makeFilenameAbsoluteFromCWD( const std::string& filename );

    //! Returns the directory for a data folder.
    //! If the environment variable TEXT_MAPPING_DATA_DIR is set, then this value will
    //! be returned. Otherwise it returns an absolute path to "../data" from the current
    //! working directory.
    //! @return An absolute path to the data directory.
    public: static std::string getDataDir();

    //! Finds the optimum transformation that minimises the squared error between two point sets
    //! where the correspondences are known. Based on the paper 'Least-squares estimation of
    //! transformation parameters between two point patterns' by Shinji Umeyama.
    //! @param pPointsA An array containing the first set of points
    //! @param pPointsB An array containing the second set of points
    //! @param numPoints The number of points in each set
    //! @param pRotationMtxOut A pointer to a matrix to be filled in with the rotation matrix
    //! @param pTranslationOut A pointer to a vector to be filled in with the translation
    //! @param pScaleInOut A pointer value to be filled in with the scale, or to pass a fixed
    //!        scale into the routine.
    //! @param bFixScale If you know what the scale should be, then set this to true, and pass
    //!        the scale in via pScaleInOut. NOTE: At the moment I've just hacked the fixed scale
    //!        in. It produces reasonable results, but I haven't checked yet to make sure that
    //!        it's still the least squares solution...
    //! @return true if it was possible to calculate the transformation, and false if the rank of
    //!        the point sets covariance was not high enough (i.e. if there weren't enough
    //!        independent points).
    //----------------------------------------------------------------------------------------------
    public: static bool findOptimumTransformation3D(
        Eigen::Vector3f* pPointsA, Eigen::Vector3f* pPointsB, uint32_t numPoints,
        Eigen::Matrix3f* pRotationMtxOut, Eigen::Vector3f* pTranslationOut, float* pScaleInOut,
        bool bFixScale=false );
};

#endif // UTILITIES_H
