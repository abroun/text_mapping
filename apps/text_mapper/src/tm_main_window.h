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
// File: tm_main_window.h
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

#ifndef MMB_MAIN_WINDOW_H_
#define MMB_MAIN_WINDOW_H_

//--------------------------------------------------------------------------------------------------
#include <ctime>
#include <list>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <QtGui/QMainWindow>
#include <QtGui/QStringListModel>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

#include <vtkOBJReader.h>
#include <vtkJPEGReader.h>
#include <vtkTexture.h>

#include "text_mapping/letter.h"
#include "ui_tm_main_window.h"
#include "image_view_dialog.h"
#include "camera.h"
#include "frame_data.h"
#include "text_mapping/vtk/vtk_point_cloud_source.h"
#include "text_mapping/text_map.h"
#include "text_mapping/vtk/vtk_text_map_source.h"

//--------------------------------------------------------------------------------------------------
class TmMainWindow : public QMainWindow, private Ui::tm_main_window
{
    Q_OBJECT

    public: TmMainWindow();
    public: virtual ~TmMainWindow();

    public slots: void onCurrentFrameChanged( const QModelIndex& current, const QModelIndex& previous );  
    public slots: void onBtnAddFrameClicked();
    public slots: void onBtnEditFrameClicked();
    public slots: void onBtnDeleteFrameClicked();
    public slots: void onBtnDetectTextClicked();
    
    public slots: void onBtnLeftClicked();
    public slots: void onBtnRightClicked();
    public slots: void onBtnUpClicked();
    public slots: void onBtnDownClicked();
    public slots: void onCheckShowModelClicked();

    private: void refreshFrameList();
    private: void refreshImageDisplays( const FrameData& frameData );

    private: typedef std::list<Letter, Eigen::aligned_allocator<Letter> > LetterList;
    private: LetterList detectTextInImage( cv::Mat image );

    private: void loadCameras();

    private: vtkSmartPointer<vtkRenderer> mpRenderer;

    // Stuff to render a point cloud
    private: vtkSmartPointer<vtkPointCloudSource> mpPointCloudSource;
    private: vtkSmartPointer<vtkCubeSource> mpCubeSource;
    private: vtkSmartPointer<vtkGlyph3D> mpPointCloudGlyphs;
    private: vtkSmartPointer<vtkPolyDataMapper> mpPointCloudMapper;
    private: vtkSmartPointer<vtkActor> mpPointCloudActor;

    // Model stuff
    // TODO: Refactor into external library
    private: void loadObjModel( QString filename );
    private: void loadTextureForModel( QString filename );

    private: vtkSmartPointer<vtkOBJReader> mpObjReader;
	private: vtkSmartPointer<vtkJPEGReader> mpModelJpegReader;
	private: vtkSmartPointer<vtkTexture> mpModelTexture;
	private: vtkSmartPointer<vtkPolyDataMapper> mpModelMapper;
	private: vtkSmartPointer<vtkActor> mpModelActor;

    // Frame list
    private: QSharedPointer<QStringListModel> mpFrameListModel;

    private: std::vector<FrameData> mFrames;
    private: ImageViewDialog mHighResImageViewDialog;
    private: ImageViewDialog mKinectColorImageViewDialog;
    private: ImageViewDialog mKinectDepthColorImageViewDialog;

    // Text map stuff
    private: vtkSmartPointer<vtkJPEGReader> mpLettersJpegReader;
    private: vtkSmartPointer<vtkTexture> mpLettersTexture;
    private: vtkSmartPointer<vtkTextMapSource> mpTextMapSource;
    private: vtkSmartPointer<vtkPolyDataMapper> mpTextMapMapper;
    private: vtkSmartPointer<vtkActor> mpTextMapActor;
    
    private: TextMap::Ptr mpFrameTextMap;

    // Cameras
    private: Camera mHighResCamera;
    private: Camera mKinectColorCamera;
    private: Camera mKinectDepthCamera;

    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // TM_MAIN_WINDOW_H_
