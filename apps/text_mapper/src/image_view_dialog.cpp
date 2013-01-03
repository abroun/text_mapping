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
// File: image_view_dialog.cpp
// Desc: Simple dialog for viewing frame images
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#include "image_view_dialog.h"
#include <QtGui/QGraphicsSceneMouseEvent>
#include "tm_main_window.h"

//--------------------------------------------------------------------------------------------------
void ImageViewPixmap::mousePressEvent( QGraphicsSceneMouseEvent *pEvent )
{
    if ( Qt::LeftButton & pEvent->buttons() )
    {
        QPointF pos = pEvent->lastPos();

        if ( pEvent->modifiers() & Qt::ControlModifier )
        {
            mpParentDialog->addKeyPointInstanceAtImagePos( pos );
        }
        else
        {
            mpParentDialog->pickFromImage( pos );
        }
    }
}

//--------------------------------------------------------------------------------------------------
const int32_t DEFAULT_MAX_IMAGE_WIDTH = 800;
const int32_t DEFAULT_MAX_IMAGE_HEIGHT = 600;

//--------------------------------------------------------------------------------------------------
// ImageViewDialog
//--------------------------------------------------------------------------------------------------
ImageViewDialog::ImageViewDialog( TmMainWindow* pParentWindow )
    : mpPixmapItem( NULL ),
      mpParentWindow( pParentWindow )
{
    setupUi( this );

    this->setWindowFlags( Qt::Window );

	// Create somewhere to show the current image
    mpScene = new QGraphicsScene( this->graphicsView );
    mpScene->setSceneRect( 0, 0, DEFAULT_MAX_IMAGE_WIDTH, DEFAULT_MAX_IMAGE_HEIGHT );
    this->graphicsView->setScene( mpScene );
}

//--------------------------------------------------------------------------------------------------
ImageViewDialog::~ImageViewDialog()
{
	if ( NULL != mpScene )
    {
        delete mpScene;
        mpScene = NULL;
    }
}

//--------------------------------------------------------------------------------------------------
void ImageViewDialog::setImage( const cv::Mat& image )
{
	if ( image.type() != CV_8UC3 && image.type() != CV_8UC4 )
	{
		printf( "Warning: Invalid image type...\n" );
		return;
	}

    // Take a copy of the image
    mImage = image;

    // Prepare for image
    mpScene->setSceneRect( 0, 0, mImage.cols, mImage.rows );

	// Draw the color image
	QPixmap pixmap( mImage.cols, mImage.rows );

	if ( image.type() == CV_8UC3 )
	{
		QImage qtImage( mImage.data, mImage.cols, mImage.rows, QImage::Format_RGB888 );
		pixmap.convertFromImage( qtImage );
	}
	else if ( image.type() == CV_8UC4 )
	{
		QImage qtImage( mImage.data, mImage.cols, mImage.rows, QImage::Format_RGB32 );
		pixmap.convertFromImage( qtImage );
	}

	if ( NULL == mpPixmapItem )
	{
	    mpPixmapItem = new ImageViewPixmap( pixmap, this );
		mpScene->addItem( mpPixmapItem );
	}
	else
	{
		mpPixmapItem->setPixmap( pixmap );
	}
}

//--------------------------------------------------------------------------------------------------
void ImageViewDialog::addKeyPointInstanceAtImagePos( const QPointF& pickPoint )
{
    mpParentWindow->addKeyPointInstanceAtImagePos( this, pickPoint );
}

//--------------------------------------------------------------------------------------------------
void ImageViewDialog::pickFromImage( const QPointF& pickPoint ) const
{
    mpParentWindow->pickFromImage( this, pickPoint );
}
