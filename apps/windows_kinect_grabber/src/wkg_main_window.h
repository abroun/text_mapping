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
// File: wkg_main_window.h
// Desc: The main window object
//--------------------------------------------------------------------------------------------------

#ifndef WKG_MAIN_WINDOW_H_
#define WKG_MAIN_WINDOW_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <ctime>
#include <string>
#include <QtGui/QMainWindow>
#include <QtGui/QGraphicsPixmapItem>
#include "ui_wkg_main_window.h"
#include <Eigen/Core>
#include <windows.h>
#include <NuiApi.h>

//--------------------------------------------------------------------------------------------------
class WkgMainWindow : public QMainWindow, private Ui::wkg_main_window
{
    Q_OBJECT

    public: WkgMainWindow();
    public: virtual ~WkgMainWindow();

	public slots: void onCheckNearModeClicked( bool bChecked = false );
	public slots: void onCbxViewCurrentIndexChanged( const QString& text );
	public slots: void onBtnGrabFrameClicked();

    private: virtual void timerEvent( QTimerEvent* pEvent ) { update(); }

    private: bool tryToConnectToKinect();
	private: HRESULT tryToSetNearMode( bool bOn );
    private: void update();
    private: HRESULT processDepth();
    private: HRESULT processColor();
	private: void mapColorToDepth();
	private: void drawImage();

	private: INuiSensor* mpNuiSensor;

    private: HANDLE mDepthStreamHandle;
    private: HANDLE mColorStreamHandle;
    private: HANDLE mNextDepthFrameEventHandle;
    private: HANDLE mNextColorFrameEventHandle;
    private: bool mbDepthReceived;
    private: bool mbColorReceived;

    private: DWORD mImageWidth;
    private: DWORD mImageHeight;
    private: uint16_t* mpDepthBuffer;
	private: uint8_t* mpDepthColorBuffer;	// For showing the depth buffer with the RGB image mapped to it
    private: uint8_t* mpColorBuffer;

    private: QGraphicsScene* mpScene;
    private: QGraphicsPixmapItem* mpPixmapItem;

	private: enum eView
	{
		eV_RgbCamera,
		eV_DepthCamera
	};

	private: eView mCurView;

    private: static const NUI_IMAGE_RESOLUTION IMAGE_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
};

#endif // WKG_MAIN_WINDOW_H_