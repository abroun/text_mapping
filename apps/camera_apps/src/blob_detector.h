//--------------------------------------------------------------------------------------------------
// File: blob_detector.h
// Desc: 
//--------------------------------------------------------------------------------------------------

#ifndef BLOB_DETECTOR_H_
#define BLOB_DETECTOR_H_

#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class BlobDetector : public cv::FeatureDetector
{
public:
  struct CV_EXPORTS_W_SIMPLE Params
  {
      CV_WRAP Params();
      CV_PROP_RW float thresholdStep;
      CV_PROP_RW float minThreshold;
      CV_PROP_RW float maxThreshold;
      CV_PROP_RW size_t minRepeatability;
      CV_PROP_RW float minDistBetweenBlobs;

      CV_PROP_RW bool filterByColor;
      CV_PROP_RW uchar blobColor;

      CV_PROP_RW bool filterByArea;
      CV_PROP_RW float minArea, maxArea;

      CV_PROP_RW bool filterByCircularity;
      CV_PROP_RW float minCircularity, maxCircularity;

      CV_PROP_RW bool filterByInertia;
      CV_PROP_RW float minInertiaRatio, maxInertiaRatio;

      CV_PROP_RW bool filterByConvexity;
      CV_PROP_RW float minConvexity, maxConvexity;

      void read( const cv::FileNode& fn );
      void write( cv::FileStorage& fs ) const;
  };

  BlobDetector(const BlobDetector::Params &parameters = BlobDetector::Params());

  virtual void read( const cv::FileNode& fn );
  virtual void write( cv::FileStorage& fs ) const;

protected:
  struct Center
  {
	  cv::Point2d location;
      double radius;
      double confidence;
  };

  virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;
  virtual void findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, std::vector<Center> &centers) const;

  Params params;
};


#endif // BLOB_DETECTOR_H_
