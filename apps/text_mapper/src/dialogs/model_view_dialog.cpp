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
// File: model_view_dialog.cpp
// Desc: A dialog for viewing a model constructed from a number of frames
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include "model_view_dialog.h"
#include <math.h>
#include <Eigen/Dense>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkOBJExporter.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "text_mapping/signed_distance_field.h"
#include "text_mapping/utilities.h"

void DumpOutput( const char* format , ... )
{
}
void DumpOutput2( char* str , const char* format , ... )
{
}

#include "poisson_surface_reconstruction/Geometry.h"
#include "poisson_surface_reconstruction/MultiGridOctreeData.h"
#include "poisson_surface_reconstruction/Octree.h"
#include "poisson_surface_reconstruction/PointStream.h"

//--------------------------------------------------------------------------------------------------
typedef std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > PointNormalVector;

//--------------------------------------------------------------------------------------------------
template< class Real >
class EigenPointStream : public PointStream< Real >
{
    PointNormalVector mPointNormalVector;
    int32_t mCurrentPointIndex;

public:
    EigenPointStream( const PointNormalVector& pointNormalVector );
    ~EigenPointStream( void );
    void reset( void );
    bool nextPoint( Point3D< Real >& p , Point3D< Real >& n );
};

//--------------------------------------------------------------------------------------------------
template< class Real >
EigenPointStream< Real >::EigenPointStream( const PointNormalVector& pointNormalVector )
    : mPointNormalVector( pointNormalVector ),
      mCurrentPointIndex( 0 )
{
}

//--------------------------------------------------------------------------------------------------
template< class Real >
EigenPointStream< Real >::~EigenPointStream( void )
{
}

//--------------------------------------------------------------------------------------------------
template< class Real >
void EigenPointStream< Real >::reset( void )
{
    mCurrentPointIndex = 0;
}

//--------------------------------------------------------------------------------------------------
template< class Real >
bool EigenPointStream< Real >::nextPoint( Point3D< Real >& p , Point3D< Real >& n )
{
    if ( mCurrentPointIndex < mPointNormalVector.size() )
    {
        const Eigen::Vector3f& pos = mPointNormalVector[ mCurrentPointIndex ].first;
        const Eigen::Vector3f& normal = mPointNormalVector[ mCurrentPointIndex ].second;

        p[0] = pos[ 0 ];
        p[1] = pos[ 1 ];
        p[2] = pos[ 2 ];
        n[0] = normal[ 0 ];
        n[1] = normal[ 1 ];
        n[2] = normal[ 2 ];

        mCurrentPointIndex++;
        return true;
    }
    else
    {
        return false;
    }
}

//--------------------------------------------------------------------------------------------------
static void texturePolyModel( const PointCloudWithPoseVector& pointCloudsAndPoses,
                              const Camera* pHighResCamera, CoredFileMeshData* pMesh );

//--------------------------------------------------------------------------------------------------
static void buildPolyModel( const PointNormalVector& pointNormalVector,
    const PointCloudWithPoseVector& pointCloudsAndPoses, const Camera* pHighResCamera )
{
    const int32_t THREADS = 4;
    const int32_t SOLVER_DIVIDE = 8;
    const int32_t ISO_DIVIDE = 8;
    const int32_t DEPTH = 5;
    const int32_t MIN_DEPTH = 5;
    const int32_t MAX_SOLVE_DEPTH = 5;
    const int32_t KERNEL_DEPTH = DEPTH - 2;
    const int32_t MIN_ITERS = 24;
    const float SAMPLES_PER_NODE = 1.0f;
    const float SCALE = 1.1f;
    const int32_t CONFIDENCE_FLAG = 0;
    const int32_t SHOW_RESIDUAL_FLAG = 0;
    const float POINT_WEIGHT = 4.0f;
    const int32_t ADAPTIVE_EXPONENT = 1;
    const int32_t BOUNDARY_TYPE = 1;
    const float SOLVER_ACCURACY = 1e-3;
    const int32_t FIXED_ITERS = -1;
    const int32_t POLYGON_MESH_FLAG = 0;
    const int32_t NON_MANIFOLD_FLAG = 0;

    EigenPointStream<float> pointStream( pointNormalVector );

    printf( "Created point stream\n" );
    fflush( stdout );

    Octree<2> tree;
    tree.threads = THREADS;

    printf( "Created tree\n" );
    fflush( stdout );

    XForm4x4<float> xForm = XForm4x4< Real >::Identity();
    XForm4x4<float> iXForm = xForm.inverse();

    printf( "Setup transform\n" );
    fflush( stdout );

    tree.setBSplineData( DEPTH, BOUNDARY_TYPE );

    printf( "set BSplineData\n" );
    fflush( stdout );

    int32_t pointCount = tree.setTree( &pointStream, DEPTH, MIN_DEPTH, KERNEL_DEPTH,
        SAMPLES_PER_NODE, SCALE, CONFIDENCE_FLAG, POINT_WEIGHT, ADAPTIVE_EXPONENT, xForm );

    printf( "Setup tree\n" );
    fflush( stdout );

    tree.ClipTree();

    printf( "Clipped tree\n" );
    fflush( stdout );

    tree.finalize( ISO_DIVIDE );

    printf( "Finalized tree\n" );
    fflush( stdout );

    tree.SetLaplacianConstraints();

    printf( "Constrained tree\n" );
    fflush( stdout );

    tree.LaplacianMatrixIteration( SOLVER_DIVIDE, SHOW_RESIDUAL_FLAG, MIN_ITERS,
        SOLVER_ACCURACY, MAX_SOLVE_DEPTH, FIXED_ITERS );

    float isoValue = tree.GetIsoValue();
    CoredFileMeshData mesh;

    tree.GetMCIsoTriangles( isoValue , ISO_DIVIDE, &mesh,
        0, 1, !NON_MANIFOLD_FLAG, POLYGON_MESH_FLAG );

    //PlyWritePolygons( "model.ply", &mesh, PLY_ASCII, NULL, 0, iXForm );

    printf( "Texturing model...\n" );
    texturePolyModel( pointCloudsAndPoses, pHighResCamera, &mesh );
}

//--------------------------------------------------------------------------------------------------
struct TexCoord
{
    TexCoord() : u( 0.0 ), v( 0.0 ) {}
    TexCoord( float u, float v ) : u( u ), v( v ) {}

    float u;
    float v;
};

//--------------------------------------------------------------------------------------------------
struct FaceElem
{
    int32_t vertexIdx;
    int32_t texCoordIdx;
};

//--------------------------------------------------------------------------------------------------
typedef std::vector<FaceElem> Face;

//--------------------------------------------------------------------------------------------------
static void texturePolyModel( const PointCloudWithPoseVector& pointCloudsAndPoses,
                              const Camera* pHighResCamera, CoredFileMeshData* pMesh )
{
    uint32_t numFrames = pointCloudsAndPoses.size();
    if ( numFrames <= 0 )
    {
        return;
    }

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > cameraMatrices;
    cameraMatrices.reserve( numFrames );

    for ( uint32_t frameIdx = 0; frameIdx < numFrames; frameIdx++ )
    {
        Eigen::Matrix4f cameraInWorldSpaceMatrix =
            pointCloudsAndPoses[ frameIdx ].mTransform*pHighResCamera->getCameraInWorldSpaceMatrix().cast<float>();

        cameraMatrices.push_back( cameraInWorldSpaceMatrix.inverse() );
    }

    // Group all of the high resolution images onto a single texture
    int32_t numImagesWide = (int32_t)ceilf( sqrtf( (float)numFrames ) );
    int32_t numImagesHigh = (int32_t)ceilf( (float)numFrames/(float)numImagesWide );

    const cv::Mat& firstImage = pointCloudsAndPoses[ 0 ].mHighResImage;
    cv::Mat combinedImage = cv::Mat::zeros(
        firstImage.rows*numImagesHigh, firstImage.cols*numImagesWide, firstImage.type() );

    for ( uint32_t frameIdx = 0; frameIdx < numFrames; frameIdx++ )
    {
        int32_t frameX = frameIdx % numImagesWide;
        int32_t frameY = frameIdx / numImagesWide;

        cv::Mat copyTarget( combinedImage, cv::Rect(
            frameX*firstImage.cols, frameY*firstImage.rows, firstImage.cols, firstImage.rows ) );
        pointCloudsAndPoses[ frameIdx ].mHighResImage.copyTo( copyTarget );
    }

    cv::cvtColor( combinedImage, combinedImage, CV_BGR2RGB );

    vector<int32_t> compressionParams;
    compressionParams.push_back( CV_IMWRITE_JPEG_QUALITY );
    compressionParams.push_back( 100 );
    cv::imwrite( "model_texture.jpg", combinedImage, compressionParams );

    // Build lists of vertices, texture cords, and faces
    std::vector<Eigen::Vector3f> vertices;
    std::vector<TexCoord> texCoords;
    std::vector<Face> faces;

    uint32_t numVertices = (uint32_t)pMesh->outOfCorePointCount() + pMesh->inCorePoints.size();
    uint32_t numFaces = (uint32_t)pMesh->polygonCount();
    printf( "Number of faces is %u\n", numFaces );

    vertices.reserve( numVertices );
    texCoords.reserve( 4*numFaces );
    faces.reserve( numFaces );

    pMesh->resetIterator();

    Point3D<float> p;
    for( uint32_t i = 0; i < pMesh->inCorePoints.size(); i++ )
    {
        p = pMesh->inCorePoints[ i ];
        vertices.push_back( Eigen::Vector3f( p[ 0 ], p[ 1 ], p[ 2 ] ) );
    }

    for( int32_t i = 0; i < pMesh->outOfCorePointCount(); i++ )
    {
        pMesh->nextOutOfCorePoint( p );
        vertices.push_back( Eigen::Vector3f( p[ 0 ], p[ 1 ], p[ 2 ] ) );
    }

    printf( "Processing %i faces\n", numFaces );


    for( uint32_t i = 0; i < numFaces; i++ )
    {
        std::vector<CoredVertexIndex> polygon;
        pMesh->nextPolygon( polygon );
        Face face;
        face.reserve( polygon.size() );

        for ( uint32_t vertexIdx = 0; vertexIdx < polygon.size(); vertexIdx++ )
        {
            FaceElem faceElem;
            faceElem.texCoordIdx = (int32_t)texCoords.size();

            if ( polygon[ vertexIdx ].inCore )
            {
                faceElem.vertexIdx = polygon[ vertexIdx ].idx;
            }
            else
            {
                faceElem.vertexIdx = polygon[ vertexIdx ].idx + (int32_t)pMesh->inCorePoints.size();
            }

            face.push_back( faceElem );
            texCoords.push_back( TexCoord() );
        }

        // Ignore degenerate faces
        if ( face.size() < 3 )
        {
            continue;
        }

        // Find out which camera has the best (most head on) view of the polygon
        const Eigen::Vector3f& v0 = vertices[ face[ 0 ].vertexIdx ];
        const Eigen::Vector3f& v1 = vertices[ face[ 1 ].vertexIdx ];
        const Eigen::Vector3f& v2 = vertices[ face[ 2 ].vertexIdx ];

        Eigen::Vector3f normal = (v2 - v1).cross( v0 - v1 );
        normal.normalize();

        int32_t mostHeadOnFrameIdx = -1;
        float cosOfMostHeadOnAngle = 0.0;   // The camera axis must point in the opposite direction
                                            // to the face normal
        for ( uint32_t frameIdx = 0; frameIdx < numFrames; frameIdx++ )
        {
            const Eigen::Vector3f& axisZ = pointCloudsAndPoses[ frameIdx ].mTransform.block<3,1>( 0, 2 );
            float cosOfAngle = normal.dot( axisZ );

            if ( cosOfAngle < cosOfMostHeadOnAngle )
            {
                mostHeadOnFrameIdx = frameIdx;
                cosOfMostHeadOnAngle = cosOfAngle;
            }
        }

        // Work out texture coordinates for the polygon
        if ( mostHeadOnFrameIdx < 0 )
        {
            fprintf( stderr, "Warning: Found a face which is not visible in any frame\n" );
        }
        else
        {
            const Eigen::Matrix3f calibMtx = pHighResCamera->getCalibrationMatrix().cast<float>();
            const Eigen::Matrix4f& camMtx = cameraMatrices[ mostHeadOnFrameIdx ];

            int32_t frameX = mostHeadOnFrameIdx % numImagesWide;
            int32_t frameY = mostHeadOnFrameIdx / numImagesWide;
            float offsetU = (float)frameX / (float)numImagesWide;
            float offsetV = (float)frameY/ (float)numImagesHigh;

            // Project each vertex onto the image plane of the frame
            for ( uint32_t faceElemIdx = 0; faceElemIdx < face.size(); faceElemIdx++ )
            {
                const FaceElem& faceElem = face[ faceElemIdx ];
                const Eigen::Vector3f& vertexWorldPos = vertices[ faceElem.vertexIdx ];
                Eigen::Vector3f camSpacePos = camMtx.block<3,3>( 0 , 0 )*vertexWorldPos + camMtx.block<3,1>( 0, 3 );

                float screenPosX = calibMtx( 0, 0 )*camSpacePos[ 0 ]/camSpacePos[ 2 ] + calibMtx( 0, 2 );
                float screenPosY = calibMtx( 1, 1 )*camSpacePos[ 1 ]/camSpacePos[ 2 ] + calibMtx( 1, 2 );

                texCoords[ faceElem.texCoordIdx ].u = offsetU + screenPosX / (float)(firstImage.cols*numImagesWide);
                texCoords[ faceElem.texCoordIdx ].v = 1.0 - (offsetV + screenPosY / (float)(firstImage.rows*numImagesHigh));
            }
        }

        faces.push_back( face );
    }

    // Write out a MTL file for the mesh
    FILE* pMtlFile = fopen( "model_material.mtl", "w" );

    fprintf( pMtlFile, "# Material Count: 1\n" );
    fprintf( pMtlFile, "newmtl Material_model_texture.jpg\n" );
    fprintf( pMtlFile, "Ns 96.078431\n" );
    fprintf( pMtlFile, "Ka 0.000000 0.000000 0.000000\n" );
    fprintf( pMtlFile, "Kd 0.640000 0.640000 0.640000\n" );
    fprintf( pMtlFile, "Ks 0.500000 0.500000 0.500000\n" );
    fprintf( pMtlFile, "Ni 1.000000\n" );
    fprintf( pMtlFile, "d 1.000000\n" );
    fprintf( pMtlFile, "illum 2\n" );
    fprintf( pMtlFile, "map_Kd model_texture.jpg\n" );
    fprintf( pMtlFile, "\n" );

    fclose( pMtlFile );

    // Write out the mesh to an OBJ file
    FILE* pObjFile = fopen( "model.obj", "w" );

    fprintf( pObjFile, "mtllib model_material.mtl\n" );
    fprintf( pObjFile, "o ObjectModel\n" );

    for ( uint32_t vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++ )
    {
        const Eigen::Vector3f& v = vertices[ vertexIdx ];
        fprintf( pObjFile, "v %f %f %f\n", v[ 0 ], v[ 1 ], v[ 2 ] );
    }

    for ( uint32_t texCoordIdx = 0; texCoordIdx < texCoords.size(); texCoordIdx++ )
    {
        const TexCoord& t = texCoords[ texCoordIdx ];
        fprintf( pObjFile, "vt %f %f\n", t.u, t.v );
    }

    fprintf( pObjFile, "usemtl Material_model_texture.jpg\n" );
    fprintf( pObjFile, "s off\n" );

    for ( uint32_t faceIdx = 0; faceIdx < faces.size(); faceIdx++ )
    {
        const Face& face = faces[ faceIdx ];
        fprintf( pObjFile, "f " );

        for ( uint32_t faceElemIdx = 0; faceElemIdx < face.size(); faceElemIdx++ )
        {
            const FaceElem& faceElem = face[ faceElemIdx ];
            fprintf( pObjFile, "%i/%i ", faceElem.vertexIdx + 1, faceElem.texCoordIdx + 1 );
        }

        fprintf( pObjFile, "\n" );
    }

    fclose( pObjFile );
}

//--------------------------------------------------------------------------------------------------
// ModelViewDialog
//--------------------------------------------------------------------------------------------------
ModelViewDialog::ModelViewDialog()
{
    setupUi( this );

    this->setWindowFlags( Qt::Window );

    // Set up a renderer and connect it to QT
	mpRenderer = vtkRenderer::New();
	qvtkWidget->GetRenderWindow()->AddRenderer( mpRenderer );

	mpRenderer->SetBackground( 0.0, 0.0, 0.0 );

	// Prepare to render model
	mpMarchingCubes = vtkMarchingCubes::New();
	mpModelMapper = vtkPolyDataMapper::New();
	mpModelActor = vtkActor::New();

	mpModelMapper->SetInput( mpMarchingCubes->GetOutput() );
	mpModelActor->SetMapper( mpModelMapper );

	mpRenderer->AddActor( mpModelActor );

	// Hook up signals
	connect( this->btnClose, SIGNAL( clicked() ), this, SLOT( onBtnCloseClicked() ) );
}

//--------------------------------------------------------------------------------------------------
ModelViewDialog::~ModelViewDialog()
{
}

//--------------------------------------------------------------------------------------------------
void ModelViewDialog::onBtnCloseClicked()
{
	close();
}

//--------------------------------------------------------------------------------------------------
void ModelViewDialog::buildModel( const PointCloudWithPoseVector& pointCloudsAndPoses, const Camera* pHighResCamera )
{
	// Clear away existing point cloud widgets
	for ( uint32_t widgetIdx = 0; widgetIdx < mPointCloudWidgets.size(); widgetIdx++ )
	{
		mpRenderer->RemoveActor( mPointCloudWidgets[ widgetIdx ].mpPointCloudActor );
	}
	mPointCloudWidgets.clear();

	// Create a point cloud widget for each point cloud
	/*for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
	{
		const PointCloudWithPose& p = pointCloudsAndPoses[ i ];

		PointCloudWidget widget;

		widget.mpPointCloudSource = vtkSmartPointer<vtkPointCloudSource>::New();
		widget.mpPointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		widget.mpPointCloudActor = vtkSmartPointer<vtkActor>::New();

		widget.mpPointCloudMapper->SetInputConnection( widget.mpPointCloudSource->GetOutputPort() );
		widget.mpPointCloudSource->SetPointCloudPtr( p.mpCloud );

		widget.mpPointCloudActor->SetMapper( widget.mpPointCloudMapper );
		widget.mpPointCloudActor->GetProperty()->SetPointSize( 3.0 );

		// Break the rotation matrix down into angles Z, X, Y. The order in which VTK will apply them
		Eigen::Vector3f angles = p.mTransform.block<3,3>( 0, 0 ).eulerAngles( 2, 0, 1 );

		// Now pass the angles as degrees to VTK
		float degreesX = Utilities::radToDeg( angles[ 1 ] );
		float degreesY = Utilities::radToDeg( angles[ 2 ] );
		float degreesZ = Utilities::radToDeg( angles[ 0 ] );
		widget.mpPointCloudActor->SetOrientation( degreesX, degreesY, degreesZ );

		const Eigen::Vector3f& pos = p.mTransform.block<3,1>( 0, 3 );
		widget.mpPointCloudActor->SetPosition( pos[ 0 ], pos[ 1 ], pos[ 2 ] );

		widget.mpPointCloudActor->SetVisibility( 1 );

		mpRenderer->AddActor( widget.mpPointCloudActor );

		mPointCloudWidgets.push_back( widget );
	}*/

	// Output all of the points to a text file
	FILE* pPointFile = fopen( "modelPoints.txt", "w" );

	PointNormalVector pointNormalVector;

	for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
    {
	    const PointCloudWithPose& p = pointCloudsAndPoses[ i ];

	    Eigen::Vector3f normal = -p.mTransform.block<3,1>( 0, 2 );

	    for ( uint32_t pointIdx = 0; pointIdx < p.mpCloud->getNumPoints(); pointIdx++ )
	    {
	        Eigen::Vector3f pos = p.mpCloud->getPointWorldPos( pointIdx );
	        pos = p.mTransform.block<3,3>( 0, 0 )*pos + p.mTransform.block<3,1>( 0, 3 );
	        fprintf( pPointFile, "%f %f %f %f %f %f\n",
	            pos[ 0 ], pos[ 1 ], pos[ 2 ], normal[ 0 ], normal[ 1 ], normal[ 2 ] );

	        pointNormalVector.push_back( std::pair<Eigen::Vector3f, Eigen::Vector3f>( pos, normal ) );
	    }
    }

    fclose( pPointFile );

    printf( "Building model...\n" );
    CoredFileMeshData mesh;
    buildPolyModel( pointNormalVector, pointCloudsAndPoses, pHighResCamera );

	if ( pointCloudsAndPoses.size() > 0 )
	{
		const float VOXEL_SIDE_LENGTH = 0.005;
		const float BOX_SIZE_X = 0.2;
		const float BOX_SIZE_Y = 0.4;
		const float BOX_SIZE_Z = 0.2;

		// Locate the centre point of the first point cloud
		Eigen::Vector3f firstCorner;
		Eigen::Vector3f secondCorner;
		pointCloudsAndPoses[ 0 ].mpCloud->getBoundingBox( &firstCorner, &secondCorner );

		Eigen::Vector3f centrePos = (firstCorner + secondCorner)/2.0f;
		Eigen::Vector3i sdfDimensions( BOX_SIZE_X/VOXEL_SIDE_LENGTH,
			BOX_SIZE_Y/VOXEL_SIDE_LENGTH, BOX_SIZE_Z/VOXEL_SIDE_LENGTH );

		// Build a Signed Distance Field (SDF) from the point clouds
		SignedDistanceField signedDistanceField( centrePos, sdfDimensions, VOXEL_SIDE_LENGTH );

		for ( uint32_t i = 0; i < pointCloudsAndPoses.size(); i++ )
		{
			printf( "Adding frame %i to SDF\n", i );
			signedDistanceField.addPointCloud(
				*(pointCloudsAndPoses[ i ].mpCloud), pointCloudsAndPoses[ i ].mTransform );
		}

		signedDistanceField.outputToVTKFile( "/home/abroun/modelSDF.vtk" );

		// Transfer the SDF to image data
		mpImageData = vtkImageData::New();
		mpImageData->SetDimensions( sdfDimensions[ 0 ], sdfDimensions[ 1 ], sdfDimensions[ 2 ] );
		mpImageData->SetScalarTypeToDouble();
		mpImageData->SetNumberOfScalarComponents( 1 );
		mpImageData->AllocateScalars();

		for ( int32_t x = 0; x < sdfDimensions[ 0 ]; x++ )
		{
			for ( int32_t y = 0; y < sdfDimensions[ 1 ]; y++ )
			{
				for ( int32_t z = 0; z < sdfDimensions[ 2 ]; z++ )
				{
					*(double*)(mpImageData->GetScalarPointer( x, y, z )) = signedDistanceField.getVoxelValue( x, y, z );
				}
			}
		}

		mpMarchingCubes->SetInput( mpImageData );

		// Use a contour filter to extract a polygonal model

		// Apply texture to the model
	}

	// Update the view
	this->qvtkWidget->update();

	vtkSmartPointer<vtkOBJExporter> pExporter = vtkSmartPointer<vtkOBJExporter>::New();
	pExporter->SetRenderWindow( qvtkWidget->GetRenderWindow() );
	pExporter->SetFilePrefix( "/home/abroun/modelMC" );
	pExporter->Write();
}


