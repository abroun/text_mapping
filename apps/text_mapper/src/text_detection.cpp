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

#ifdef WIN32
#include <Windows.h>

#include <tesseract/baseapi.h>
#include <tesseract/resultiterator.h> 
#endif

struct CompStruct
{
	CvRect boundingBox;
	std::string textString;
};

int detect_text(cv::Mat inputImage)
{   
    cv::cvtColor(inputImage,inputImage,cv::COLOR_RGB2BGR);

#ifdef WIN32	
	cv::Mat outputImage = inputImage.clone();
	tesseract::TessBaseAPI tess;
	tess.Init("C:\\Program Files (x86)\\Tesseract-OCR\\tessdata", "eng", tesseract::OEM_DEFAULT);
	
	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_COMPLEX_SMALL, .6, .6, 0, 1, 6);

	std::vector<CompStruct> good;
	std::vector<CompStruct> bad;
	CompStruct storeComp;
	//list.push_back(cv::Rect(x,y,dx,dy));
	storeComp.boundingBox = cv::Rect(1208,581,163,56);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1192,645,189,46);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1103,1402,174,21);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1035,1532,301,26);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(1381,1489,65,36);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(888,293,103,110);
	good.push_back(storeComp);
	storeComp.boundingBox = cv::Rect(917,394,42,38);
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
	bad.push_back(storeComp);
	


	for(int i = 0; i < good.size(); i++)
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
	}

	for(int i = 0; i < bad.size(); i++)
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
	for(int i = 0; i < good.size(); i++)
	{
		std::cout << good.at(i).textString;// << std::endl;
		cv::rectangle(outputImage,good.at(i).boundingBox,cv::Scalar(0,255,255),2);
		cv::putText(outputImage,good.at(i).textString,cv::Point(good.at(i).boundingBox.x,good.at(i).boundingBox.y),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,0,255));
	}

	std::cout << "Unaccepted text! " << std::endl;

	for(int i = 0; i < bad.size(); i++)
	{
		std::cout << bad.at(i).textString;// << std::endl;
		cv::rectangle(outputImage,bad.at(i).boundingBox,cv::Scalar(0,0,255),2);
	}

	
	cv::resize(outputImage,outputImage,cv::Size(1024,768));
	cv::imshow("out",outputImage);
	cv::waitKey();
	cv::destroyAllWindows();

#endif
	return 0;
}
