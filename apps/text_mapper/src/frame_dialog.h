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
// File: frame_dialog.h
// Desc: A dialog which allows the user to set the properties of a frame
//--------------------------------------------------------------------------------------------------

#ifndef FRAME_DIALOG_H_
#define FRAME_DIALOG_H_

//--------------------------------------------------------------------------------------------------
#include <QtGui/QDialog>
#include "ui_frame_dialog.h"
#include "frame_data.h"

//--------------------------------------------------------------------------------------------------
class FrameDialog : public QDialog, private Ui::frame_dialog
{
    Q_OBJECT

    public: FrameDialog();
    public: virtual ~FrameDialog();

    public: static bool createNewFrame( FrameData* pFrameDataOut );
	public: static void editFrame( FrameData* pFrameDataInOut );

	public slots: void onBtnChooseHighResImageClicked();
	public slots: void onBtnChooseKinectColorImageClicked();
	public slots: void onBtnChooseKinectDepthPointCloudClicked();
	
    protected: virtual void accept();

	private: void setFrameData( const FrameData& frameData );
	private: const FrameData& getFrameData() const { return mFrameData; }
	
	private: FrameData mFrameData;
};

#endif // FRAME_DIALOG_H_
