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
// File: text_detection.cpp
// Desc: main text_detection routines
//--------------------------------------------------------------------------------------------------

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <Windows.h>

#include <baseapi.h>
#include <resultiterator.h> 

void drawMSER(cv::Mat &image, CvSeq *seq, int level = 0) 
{
	   cv::Vec3b color_vec;
	   int color_val = 50 + level * 205 / 5;
	   if (((CvContour *)seq)->color >= 0)
	      color_vec = cv::Vec3b(0,0, color_val);
	   else
	      color_vec = cv::Vec3b(color_val,0,0);
	
	   for (int j = 0; j < seq->total; ++j) {
	      CvPoint *pos = CV_GET_SEQ_ELEM(CvPoint, seq, j);
	      image.at<cv::Vec3b>(pos->y, pos->x) = color_vec;
	   }
	
	   // Walk all the children of this node.
	   CvSeq *it = seq->v_next;
	   while (it) {
	      drawMSER(image, it, level + 1);
	      it = it->h_next;
	   }
	}

int detect_text(cv::Mat inputImage)
{
	cv::cvtColor(inputImage,inputImage,cv::COLOR_RGB2BGR);

	cv::Mat GrayImage = inputImage.clone();
	cv::Mat draw_image = inputImage.clone();

	cv::cvtColor(GrayImage,GrayImage,cv::COLOR_RGB2GRAY);

	CvMSERParams MSERparams = cvMSERParams(5,40,5000);
	CvSeq* MSERegions;
	CvMemStorage* MSER_Storage = cvCreateMemStorage();

	cvExtractMSER(&GrayImage.operator IplImage(),NULL, &MSERegions,MSER_Storage,MSERparams);

	for (size_t i = 0; i < MSERegions->total; ++i) 
	{
	      CvSeq *seq = *CV_GET_SEQ_ELEM(CvSeq *, MSERegions, i);
	      // No parent, so it is a root node.
	      if (seq->v_prev == NULL)
	         drawMSER(draw_image, seq);
	}

	tesseract::TessBaseAPI tess;
	tess.Init("C:\\Program Files (x86)\\Tesseract-OCR\\tessdata", "eng", tesseract::OEM_DEFAULT);
	tess.SetImage((uchar*)inputImage.data, inputImage.size().width, inputImage.size().height, inputImage.channels(),inputImage.step1());
	tess.Recognize(0);
	std::cout << tess.GetUTF8Text() << std::endl;

	cv::resize(draw_image,draw_image,cv::Size(1024,768));
	cv::imshow("out",draw_image);
	cv::waitKey();
	cv::destroyAllWindows();

	return 0;
}