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
// File: image_view_dialog.h
// Desc: Simple dialog for viewing frame images
//--------------------------------------------------------------------------------------------------

#ifndef IMAGE_VIEW_DIALOG_H_
#define IMAGE_VIEW_DIALOG_H_

//--------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <QtGui/QDialog>
#include <QtGui/QMainWindow>
#include <QtGui/QGraphicsPixmapItem>
#include <opencv2/core/core.hpp>
#include "ui_image_view_dialog.h"
#include "key_point.h"

class TmMainWindow;
class ImageViewDialog;

//--------------------------------------------------------------------------------------------------
class ImageViewPixmap : public QGraphicsPixmapItem
{
    public: ImageViewPixmap( const QPixmap &pixmap, ImageViewDialog* pParentDialog )
        : QGraphicsPixmapItem( pixmap ), mpParentDialog( pParentDialog ), mbDraggingRectangle( false ) {}

    public: virtual void mousePressEvent( QGraphicsSceneMouseEvent *pEvent );
    public: virtual void mouseMoveEvent( QGraphicsSceneMouseEvent *pEvent );
    public: virtual void mouseReleaseEvent( QGraphicsSceneMouseEvent *pEvent );

    private: void updateDragRectangle( const QPointF& curPos );

    private: ImageViewDialog* mpParentDialog;
    private: bool mbDraggingRectangle;
    private: QPointF mDragStartPoint;
    private: QRectF mDragRectangle;
};

//--------------------------------------------------------------------------------------------------
class ImageViewDialog : public QDialog, private Ui::image_view_dialog
{
    Q_OBJECT

    public: ImageViewDialog( TmMainWindow* pParentWindow );
    public: virtual ~ImageViewDialog();
	
    public slots: void onBtnSetFilterClicked();

	public: void setImage( const cv::Mat& image );
	public: void setKeyPointInstancesToDisplay(
		const std::vector<KeyPointInstanceData>& keyPointInstances );

	public: void addKeyPointInstanceToFrameAtImagePos( const QPointF& pickPoint );
	public: void pickFromImage( const QPointF& pickPoint ) const;

	public: void setDragRectangle( const QRectF& dragRectangle );
	public: void clearDragRectangle();

	private: QGraphicsScene* mpScene;
    private: QGraphicsPixmapItem* mpPixmapItem;
    private: cv::Mat mImage;
    private: TmMainWindow* mpParentWindow;
    private: std::vector<KeyPointInstanceData> mKeyPointInstances;
    private: std::vector<QGraphicsItem*> mKeyPointGraphicItems;

    private: QRectF mDragRectangle;
    private: bool mbDragRectangleSet;
    private: QGraphicsRectItem* mpDragRectangleItem;
};

#endif // IMAGE_VIEW_DIALOG_H_
