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
// File: mmb_main_window.h
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

#ifndef MMB_MAIN_WINDOW_H_
#define MMB_MAIN_WINDOW_H_

//--------------------------------------------------------------------------------------------------
#include <string>
#include <QtGui/QMainWindow>
#include <vtkRenderer.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkJPEGReader.h>
#include <vtkTexture.h>
#include "ui_mmb_main_window.h"
#include <Eigen/Core>
#include "text_mapping/text_map.h"
#include "text_mapping/vtk/vtk_text_map_source.h"

//--------------------------------------------------------------------------------------------------
class MmbMainWindow : public QMainWindow, private Ui::mmb_main_window
{
    Q_OBJECT

    public: MmbMainWindow();
    public: virtual ~MmbMainWindow();

    public slots: void onNew();
    public slots: void onOpen();
    public slots: void onSave();
    public slots: void onSaveAs();
    public slots: void onSetModel();
    
    private: void loadObjModel( QString filename );
    private: void loadTextureForModel( QString filename );
    
    private: vtkSmartPointer<vtkRenderer> mpRenderer;

    private: vtkSmartPointer<vtkOBJReader> mpObjReader;
    private: vtkSmartPointer<vtkJPEGReader> mpModelJpegReader;
    private: vtkSmartPointer<vtkTexture> mpModelTexture;
    private: vtkSmartPointer<vtkPolyDataMapper> mpModelMapper;
    private: vtkSmartPointer<vtkActor> mpModelActor;
    
    private: vtkSmartPointer<vtkJPEGReader> mpLettersJpegReader;
    private: vtkSmartPointer<vtkTexture> mpLettersTexture;
    private: vtkSmartPointer<vtkTextMapSource> mpTextMapSource;
    private: vtkSmartPointer<vtkPolyDataMapper> mpTextMapMapper;
    private: vtkSmartPointer<vtkActor> mpTextMapActor;
    
    private: TextMap::Ptr mpModelTextMap;
    private: Eigen::Matrix4f mTransformToTarget;
    private: std::string mTextMapFilename;
    
    public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // MMB_MAIN_WINDOW_H_
