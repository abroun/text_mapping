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

#include <stdint.h>
#include "text_detection.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <boost/algorithm/string.hpp>

#ifdef WIN32
#include <Windows.h>
#endif

#include <tesseract/baseapi.h>
#include <tesseract/resultiterator.h> 


struct CompStruct
{
	CvRect boundingBox;
	std::string textString;
};

void ExtractMSER(cv::Mat GrayImage,CvSeq *allMSERregions);
void CleanCompList(CvSeq *seq,cv::Mat CannyImage,cv::Mat EdgeOriImage,cv::Mat &LightComps,cv::Mat &DarkComps);
void ProduceEdgeImage(cv::Mat Image,cv::Mat &CannyImage,cv::Mat &EdgeOriImage);
void CleanComp(CvSeq *seq,cv::Mat CannyImage,cv::Mat EdgeOriImage,cv::Mat &OutputImage);
double round(double r);
void labelBlobs(const cv::Mat &binary, std::vector <std::vector<cv::Point>> &blobs);

std::vector<CompStruct> extractTextRectangles( cv::Mat InputImage );

Letter2DVector detect_text(cv::Mat inputImage, uint32_t frameIdx )
{   
    Letter2DVector foundLetters;

    cv::cvtColor(inputImage,inputImage,cv::COLOR_RGB2BGR);


	cv::Mat outputImage = inputImage.clone();
	tesseract::TessBaseAPI tess;

#ifdef WIN32
	tess.Init("C:\\Program Files (x86)\\Tesseract-OCR\\tessdata", "eng", tesseract::OEM_DEFAULT);
#else
	tess.Init( "/usr/share/tesseract-ocr/tessdata/", "eng", tesseract::OEM_DEFAULT );
#endif
	
	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_COMPLEX_SMALL, .6, .6, 0, 1, 6);

	std::vector<CompStruct> textRectangles = extractTextRectangles( inputImage );

	std::vector<CompStruct> good;
	std::vector<CompStruct> bad;
	CompStruct storeComp;
	//list.push_back(cv::Rect(x,y,dx,dy));
	//storeComp.boundingBox = cv::Rect(713,462,533,156);
	//good.push_back(storeComp);
	
	/*if ( 2 == frameIdx )
	{
		storeComp.boundingBox = cv::Rect(1077,458,105,28);
		good.push_back(storeComp);
		storeComp.boundingBox = cv::Rect(1082,418,84,38);
		good.push_back(storeComp);
		storeComp.boundingBox = cv::Rect(1221,422,193,107);
		good.push_back(storeComp);
	}
	if ( 1 == frameIdx )
	{
		storeComp.boundingBox = cv::Rect(1020,351,290,200);
		good.push_back(storeComp);
	}
	else // 0 or otherwise*/
	{
		storeComp.boundingBox = cv::Rect(869,632,223,61);
		good.push_back(storeComp);
		storeComp.boundingBox = cv::Rect(860,702,251,54);
		good.push_back(storeComp);
		storeComp.boundingBox = cv::Rect(935,411,104,19);
		good.push_back(storeComp);

		storeComp.boundingBox = cv::Rect(888,429,199,15);
		good.push_back(storeComp);
		storeComp.boundingBox = cv::Rect(950,445,75,15);
		good.push_back(storeComp);
	}
	
    /*storeComp.boundingBox = cv::Rect(917,394,42,38);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1044,1444,278,22);
	good.push_back(storeComp);

	storeComp.boundingBox = cv::Rect(852,568,165,285);
	bad.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(912,428,47,33);
	bad.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1229,396,150,43);
	bad.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1086,392,403,186);
	bad.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1110,1467,217,24);
	bad.push_back(storeComp);*/
	


	for(uint32_t i = 0; i < good.size(); i++)
	{

		cv::Mat storeImage = cv::Mat::zeros(good.at(i).boundingBox.height,good.at(i).boundingBox.width, inputImage.type());

		for(int ypos = 0; ypos < good.at(i).boundingBox.height; ypos++)
		{
			for(int xpos = 0; xpos < good.at(i).boundingBox.width; xpos++)
			{
				storeImage.at<cv::Vec3b>(ypos,xpos) = inputImage.at<cv::Vec3b>(good.at(i).boundingBox.y+ypos,good.at(i).boundingBox.x+xpos);
			}
		}

		tess.Clear();
		tess.SetImage((uchar*)storeImage.data, storeImage.size().width, storeImage.size().height, storeImage.channels(),storeImage.step1());
		tess.Recognize(0);
		good.at(i).textString = tess.GetUTF8Text();

        // Convert the string into letters. Assume that each letter takes up the same amount of space
        // in the rectangle...
        std::string textString = good.at(i).textString;
        boost::algorithm::trim( textString );

        if ( textString.length() > 0 )
        {
            float topLeftX = (float)good.at(i).boundingBox.x;
            float letterWidth = (float)good.at(i).boundingBox.width / (float)textString.length();
            float letterTopY = (float)good.at(i).boundingBox.y;
            float letterBottomY = letterTopY + (float)good.at(i).boundingBox.height;

            storeComp.boundingBox = cv::Rect(713,462,533,156);

            for ( uint32_t letterIdx = 0; letterIdx < textString.length(); letterIdx++ )
            {
                float letterLeftX = topLeftX + letterIdx*letterWidth;
                float letterRightX = topLeftX + (letterIdx + 1)*letterWidth;

                Letter2D letter2D;
                letter2D.mCharacter = textString[ letterIdx ];
                letter2D.mTopLeft = Eigen::Vector2d( letterLeftX, letterTopY );
                letter2D.mTopRight = Eigen::Vector2d( letterRightX, letterTopY );
                letter2D.mBottomLeft = Eigen::Vector2d( letterLeftX, letterBottomY );
                letter2D.mBottomRight = Eigen::Vector2d( letterRightX, letterBottomY );

                foundLetters.push_back( letter2D );
            }
        }
	}

	for(uint32_t i = 0; i < bad.size(); i++)
	{

		cv::Mat storeImage = cv::Mat::zeros(bad.at(i).boundingBox.height,bad.at(i).boundingBox.width, inputImage.type());

		for(int ypos = 0; ypos < bad.at(i).boundingBox.height; ypos++)
		{
			for(int xpos = 0; xpos < bad.at(i).boundingBox.width; xpos++)
			{
				storeImage.at<cv::Vec3b>(ypos,xpos) = inputImage.at<cv::Vec3b>(bad.at(i).boundingBox.y+ypos,bad.at(i).boundingBox.x+xpos);
			}
		}

		tess.Clear();
		tess.SetImage((uchar*)storeImage.data, storeImage.size().width, storeImage.size().height, storeImage.channels(),storeImage.step1());
		tess.Recognize(0);
		bad.at(i).textString = tess.GetUTF8Text();
	}

	std::cout << "Accepted text! " << std::endl;
	for(uint32_t i = 0; i < good.size(); i++)
	{
		std::cout << good.at(i).textString;// << std::endl;
		cv::rectangle(outputImage,good.at(i).boundingBox,cv::Scalar(0,255,255),2);
		cv::putText(outputImage,good.at(i).textString,cv::Point(good.at(i).boundingBox.x,good.at(i).boundingBox.y),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,0,255));
	}

	std::cout << "Unaccepted text! " << std::endl;

	for(uint32_t i = 0; i < bad.size(); i++)
	{
		std::cout << bad.at(i).textString;// << std::endl;
		cv::rectangle(outputImage,bad.at(i).boundingBox,cv::Scalar(0,0,255),2);
	}

	
	cv::resize(outputImage,outputImage,cv::Size(1024,768));
	cv::imshow("out",outputImage);
	cv::waitKey();
	cv::destroyAllWindows();

	return foundLetters;
}

void ExtractMSER(cv::Mat GrayImage,CvSeq *allMSERregions)
{
	cv::Ptr<CvMemStorage> storage(cvCreateMemStorage(0));
	CvMSERParams params = cvMSERParams(5,50,6100);
	cvExtractMSER(&(IplImage)GrayImage, NULL, &allMSERregions, storage, params);
}

void ProduceEdgeImage(cv::Mat Image,cv::Mat &CannyImage,cv::Mat &EdgeOriImage)
{
	blur(Image, CannyImage, cv::Size(3,3) );

	cv::Canny(CannyImage,CannyImage,30,90);

    cv::Mat Sx;
    Sobel(Image, Sx, CV_32F, 1, 0, 3);

    cv::Mat Sy;
    Sobel(Image, Sy, CV_32F, 0, 1, 3);

    cv::Mat mag;
    magnitude(Sx, Sy, mag);
    phase(Sx, Sy, EdgeOriImage, true);

	cv::imwrite("C:\\Users\\Chris\\Desktop\\Fast Detection\\data\\Output\\Canny.png",CannyImage);
}

void CleanCompList(CvSeq *allMSERregions,cv::Mat CannyImage,cv::Mat EdgeOriImage,cv::Mat &LightComps,cv::Mat &DarkComps)
{
	CvSeq *currentseq;
	CvSeq *nextseq;
	for(size_t i = 0; i < allMSERregions->total; i++)
	{
		currentseq = *CV_GET_SEQ_ELEM(CvSeq *, allMSERregions, i);
		if (currentseq->v_prev == NULL) // Only run if parent node
		{
			/*if(currentseq->v_next != NULL && currentseq->v_next->h_next ==NULL)
			{
				currentseq = currentseq->v_next;
			}*/
			if (((CvContour *)currentseq)->color >= 0)
			{

					CleanComp(currentseq,CannyImage,EdgeOriImage,LightComps);

			}
			else
			{
					CleanComp(currentseq,CannyImage,EdgeOriImage,DarkComps);
			}
		}
	}

}

void CleanComp(CvSeq *seq,cv::Mat CannyImage,cv::Mat EdgeOriImage,cv::Mat &OutputImage)
{
	CvPoint CurrentPoint,*LocalPoint;
	float Angle = 0;
	CvRect BoundingBox = cvBoundingRect(seq,1);

	cv::Mat StoreImage = cv::Mat::zeros(BoundingBox.height,BoundingBox.width, CV_8UC3);

	for (size_t j = 0; j < seq->total; ++j)
	{
		LocalPoint = CV_GET_SEQ_ELEM(CvPoint, seq, j);
		StoreImage.at<cv::Vec3b>(LocalPoint->y-BoundingBox.y, LocalPoint->x-BoundingBox.x) = cv::Vec3b(255,255,255);
	}

	int searchDirection = 0;
	if (((CvContour *)seq)->color >= 0)
		searchDirection = 1;
	else
		searchDirection = -1;

	for (int j = 0; j < seq->total; ++j)
	{
		LocalPoint = CV_GET_SEQ_ELEM(CvPoint, seq, j);
		CurrentPoint.x = LocalPoint->x-BoundingBox.x;
		CurrentPoint.y = LocalPoint->y-BoundingBox.y;

		float step = 1;
		int currY = CurrentPoint.y;
		int currX = CurrentPoint.x;

		if(cvGet2D(&(IplImage)CannyImage,CurrentPoint.y+BoundingBox.y,CurrentPoint.x+BoundingBox.x).val[0] == 255)
		{
			Angle = cvGet2D(&(IplImage)EdgeOriImage,CurrentPoint.y+BoundingBox.y,CurrentPoint.x+BoundingBox.x).val[0];

			while (step<5)
			{
				int nextY = round(CurrentPoint.y + sin(Angle*(M_PI/180))*searchDirection*step);
				int nextX = round(CurrentPoint.x + cos(Angle*(M_PI/180))*searchDirection*step);

				if (nextY < 0 || nextX < 0 ||
					nextY >= BoundingBox.height||
					nextX >= BoundingBox.width)
					break;

				step = step + 1;
				if (currY == nextY && currX == nextX)
				continue;

				currY = nextY;
				currX = nextX;

				if(cvGet2D(&(IplImage)CannyImage,currY+BoundingBox.y,currX+BoundingBox.x).val[0] == 255)
				 break;

				StoreImage.at<cv::Vec3b>(currY,currX) = cv::Vec3b(0,0,0);

			}
		}
	}

	for (int j = 0; j < seq->total; ++j)
	{
		LocalPoint = CV_GET_SEQ_ELEM(CvPoint, seq, j);
		if(cvGet2D(&(IplImage)StoreImage,LocalPoint->y-BoundingBox.y,LocalPoint->x-BoundingBox.x).val[0] != 0)
		{
				OutputImage.at<cv::Vec3b>(LocalPoint->y,LocalPoint->x) = cv::Vec3b(255,255,255);
		}
	}
}

double round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

std::vector<CompStruct> extractTextRectangles( cv::Mat InputImage )
{
	std::vector<CompStruct> textRectangles;

	cv::Mat GrayImage, DarkComps, LightComps,CannyImage,EdgeOriImage,blank;
	cv::cvtColor( InputImage, GrayImage, CV_RGB2GRAY );

	DarkComps = cv::Mat::zeros(GrayImage.size().height,GrayImage.size().width, CV_8UC3);
	LightComps = cv::Mat::zeros(GrayImage.size().height,GrayImage.size().width, CV_8UC3);
	blank = cv::Mat::zeros(GrayImage.size().height,GrayImage.size().width, CV_8UC3);

	ProduceEdgeImage(GrayImage,CannyImage,EdgeOriImage);

	cv::Ptr<CvMemStorage> storage(cvCreateMemStorage(0));
	CvMSERParams params = cvMSERParams(5,50,6100);
	CvSeq *allMSERregions;
	cvExtractMSER(&(IplImage)GrayImage, NULL, &allMSERregions, storage, params);

	CleanCompList(allMSERregions,CannyImage,EdgeOriImage,LightComps,DarkComps);


	cv::namedWindow( "LightComps", CV_WINDOW_NORMAL );
	cv::namedWindow( "DarkComps", CV_WINDOW_NORMAL );

	cv::imshow( "LightComps", LightComps );
	cv::imshow( "DarkComps", DarkComps );

	cv::waitKey();

	GrayImage.release(), DarkComps.release(), LightComps.release();

	return textRectangles;
}
