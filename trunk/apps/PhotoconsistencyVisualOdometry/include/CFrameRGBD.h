/*
 *  Photoconsistency-Visual-Odometry
 *  Multiscale Photoconsistency Visual Odometry from RGBD Images
 *  Copyright (c) 2012, Miguel Algaba Borrego
 *
 *  http://code.google.com/p/photoconsistency-visual-odometry/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CFRAME_RGBD
#define CFRAME_RGBD

#define ENABLE_POINTCLOUD_DOWNSAMPLER 1
#define ENABLE_OPENMP_MULTITHREADING 0

#if ENABLE_POINTCLOUD_DOWNSAMPLER
    #include "PointCloudDownsampler.h"
#else
    #include <pcl/filters/voxel_grid.h>
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <boost/lexical_cast.hpp>

#include "third_party/cvmat_serialization.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>

/*!
The class CFrameRGBD encapsulates the RGB and depht data of a certain RGBD frame. It also contains the timestamp of the RGBD data.
*/
class CFrameRGBD
{
private:

  /*!RGB image*/
  cv::Mat m_rgbImage;

  /*!Intensity image (grayscale version of the RGB image)*/
  cv::Mat m_intensityImage;

  /*!Depth image*/
  cv::Mat m_depthImage;

  /*!Coloured point cloud*/
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_pointCloudPtr;

  /*!Downsampled coloured point cloud*/
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_downsampledPointCloudPtr;

  /*!True if the coloured point cloud is available, false otherwise*/
  bool pointCloudAvailable;

  /*!True if the downsampled coloured point cloud is available, false otherwise*/
  bool downsampledPointCloudAvailable;

  /*!True if the intensity image is available, false otherwise*/
  bool intensityImageAvailable;

  /*!Timestamp of the RGBD frame*/
  uint64_t m_timeStamp;

  /*!Max pointcloud depth*/
  float maxDepth;

  /*!Min pointcloud depth*/
  float minDepth;

public:

  CFrameRGBD()
  {
      pointCloudAvailable=false;
      downsampledPointCloudAvailable=false;
      intensityImageAvailable=false;

      minDepth = 0.3; // Default min depth
      maxDepth = 5.0; // Default max depth
  };

  ~CFrameRGBD(){};

  /*!Sets a RGB image to the RGBD frame.*/
  inline void setRGBImage(const cv::Mat & rgbImage){m_rgbImage = rgbImage;}

  /*!Returns the RGB image of the RGBD frame.*/
  inline cv::Mat & getRGBImage(){return m_rgbImage;}

  /*!Sets a depth image to the RGBD frame.*/
  inline void setDepthImage(const cv::Mat & depthImage){m_depthImage = depthImage;}

  /*!Returns the depth image of the RGBD frame.*/
  inline cv::Mat & getDepthImage(){return m_depthImage;}

  /*!Set the RGBD frame timestamp*/
  inline void setTimeStamp(uint64_t timeStamp){m_timeStamp=timeStamp;};

  /*!Returns the RGBD frame timestamp*/
  inline uint64_t getTimeStamp(){return m_timeStamp;};

  /*!Gets a grayscale version of the RGB image*/
  inline cv::Mat & getIntensityImage()
  {
      //If the intensity image has been already computed, don't compute it again
      if(!intensityImageAvailable)
      {
          cv::cvtColor(m_rgbImage,m_intensityImage,CV_BGR2GRAY);

          //The intensity image is now available
          intensityImageAvailable = true;
      }

      return m_intensityImage;
  }

  /*!Sets the max depth value for the point cloud points.*/
  inline void setMaxPointCloudDepth(float maxD)
  {
        maxDepth = maxD;
  }

  /*!Sets the min depth value for the point cloud points.*/
  inline void setMinPointCloudDepth(float minD)
  {
        minDepth = minD;
  }

  /*!Returns the max depth value for point cloud points.*/
  inline float getMaxPointCloudDepth()
  {
      return maxDepth;
  }

  /*!Returns the min depth value for point cloud points.*/
  inline float getMinPointCloudDepth()
  {
      return minDepth;
  }

  /*!Gets a 3D coloured point cloud from the RGBD data using the camera parameters*/
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(const Eigen::Matrix3f & cameraMatrix)
  {
    //If the point cloud has been already computed, don't compute it again
    if(!pointCloudAvailable)
    {
        const float inv_fx = 1.f/cameraMatrix(0,0);
        const float inv_fy = 1.f/cameraMatrix(1,1);
        const float ox = cameraMatrix(0,2);
        const float oy = cameraMatrix(1,2);

        int height = m_rgbImage.rows;
        int width = m_rgbImage.cols;

        m_pointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
        m_pointCloudPtr->height = height;
        m_pointCloudPtr->width = width;
        m_pointCloudPtr->is_dense = false;
        m_pointCloudPtr->points.resize(height*width);

        #if ENABLE_OPENMP_MULTITHREADING
        #pragma omp parallel for
        #endif
        /*for( int y = 0; y < height; y++ )
        {
            for( int x = 0; x < width; x++ )
            {
                float z = m_depthImage.at<float>(y,x); //convert from milimeters to meters
                m_pointCloudPtr->points[width*y+x].x = z;
                m_pointCloudPtr->points[width*y+x].y = -(x - ox) * z * inv_fx;
                m_pointCloudPtr->points[width*y+x].z = -(y - oy) * z * inv_fy;
                cv::Vec3b& bgr = m_rgbImage.at<cv::Vec3b>(y,x);
                m_pointCloudPtr->points[width*y+x].r = bgr[2];
                m_pointCloudPtr->points[width*y+x].g = bgr[1];
                m_pointCloudPtr->points[width*y+x].b = bgr[0];
            }
        }*/
        for( int y = 0; y < height; y++ )
        {
            for( int x = 0; x < width; x++ )
            {
                float z = m_depthImage.at<float>(y,x); //convert from milimeters to meters
                if(z>0 && z>=minDepth && z<=maxDepth) //If the point has valid depth information assign the 3D point to the point cloud
                {
                    m_pointCloudPtr->points[width*y+x].x = z;
                    m_pointCloudPtr->points[width*y+x].y = -(x - ox) * z * inv_fx;
                    m_pointCloudPtr->points[width*y+x].z = -(y - oy) * z * inv_fy;
                    cv::Vec3b& bgr = m_rgbImage.at<cv::Vec3b>(y,x);
                    m_pointCloudPtr->points[width*y+x].r = bgr[2];
                    m_pointCloudPtr->points[width*y+x].g = bgr[1];
                    m_pointCloudPtr->points[width*y+x].b = bgr[0];
                }
                else //else, assign a NAN value
                {
                    m_pointCloudPtr->points[width*y+x].x = std::numeric_limits<float>::quiet_NaN ();
                    m_pointCloudPtr->points[width*y+x].y = std::numeric_limits<float>::quiet_NaN ();
                    m_pointCloudPtr->points[width*y+x].z = std::numeric_limits<float>::quiet_NaN ();
                }
            }
        }

        //The point cloud is now available
        pointCloudAvailable = true;
    }

    return m_pointCloudPtr;

  }

  /*!Gets a downsampled version of the RGB point cloud*/
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getDownsampledPointCloud(const Eigen::Matrix3f & cameraMatrix)
  {
    //If the downsampled point cloud has been already computed, don't compute it again
    if(!downsampledPointCloudAvailable)
    {
        m_downsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
        #if ENABLE_POINTCLOUD_DOWNSAMPLER
            PointCloudDownsampler grid;
            grid = PointCloudDownsampler(8);
            grid.setMaximumDepth(maxDepth);
            grid.setMinimumDepth(minDepth);
            grid.downsamplePointCloudColor(getPointCloud(cameraMatrix),m_downsampledPointCloudPtr);
        #else
            pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
            grid.setLeafSize(0.04,0.04,0.04);
            grid.setFilterFieldName ("x");
            grid.setFilterLimits (0.3,5.0);
            grid.setInputCloud (getPointCloud(cameraMatrix));
            grid.filter(*m_downsampledPointCloudPtr);
        #endif

        //The downsampled point cloud is now available
        downsampledPointCloudAvailable = true;
    }

    return m_downsampledPointCloudPtr;

  }

  void getMatrixNumberRepresentationOf_uint64_t(uint64_t number,cv::Mat & matrixNumberRepresentation)
  {

      //Determine the number of digits of the number
      int num_digits = 0;
      uint64_t number_aux = number;
      while(number_aux > 0)
      {
        num_digits++;
        number_aux/=10;
      }

      //Compute the matrix representation of the number
      matrixNumberRepresentation = cv::Mat::zeros(1,num_digits,CV_8U);

      uint64_t remainder = number;
      for(int digitIndex=0;digitIndex<num_digits;digitIndex++)
      {
        if(remainder==0){break;}

        uint8_t greaterDigit;
        uint64_t dividend = remainder;
        uint64_t divisor = pow(10,num_digits-1-digitIndex);
        uint64_t quotient = remainder / divisor;
        greaterDigit = quotient;
        matrixNumberRepresentation.at<uint8_t>(0,digitIndex)=greaterDigit;
        remainder = dividend - divisor * quotient;
      }
  }

  void saveToFile(std::string fileName)
  {
    std::ofstream ofs(fileName.append(".bin").c_str(), std::ios::out | std::ios::binary);

    {   // use scope to ensure archive goes out of scope before stream

      boost::archive::binary_oarchive oa(ofs);

      cv::Mat timeStampMatrix;
      getMatrixNumberRepresentationOf_uint64_t(m_timeStamp,timeStampMatrix);
      oa << m_depthImage << m_rgbImage << timeStampMatrix;
    }

    ofs.close();

  }

  void get_uint64_t_ofMatrixRepresentation(cv::Mat & matrixNumberRepresentation,uint64_t & number)
  {
      int num_digits = matrixNumberRepresentation.cols;

      number=0;
      uint64_t power10=1;
      for(int digitIndex=num_digits-1;digitIndex>=0;digitIndex--)
      {
          number += power10 * uint64_t(matrixNumberRepresentation.at<uint8_t>(0,digitIndex));

          power10 = power10 * 10;
      }
  }

  void loadFromFile(std::string fileName)
  {
      std::ifstream ifs(fileName.append(".bin").c_str(), std::ios::in | std::ios::binary);

      { // use scope to ensure archive goes out of scope before stream

        boost::archive::binary_iarchive ia(ifs);

        cv::Mat timeStampMatrix;
        ia >> m_depthImage >> m_rgbImage >> timeStampMatrix;
        get_uint64_t_ofMatrixRepresentation(timeStampMatrix,m_timeStamp);
      }

      ifs.close();

      //Initialize the intensity image with an empty matrix
      m_intensityImage = cv::Mat();
      intensityImageAvailable = false; //The intensity image is initially unavailable

      //Initialize the point cloud with an empty pointer
      m_pointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
      pointCloudAvailable = false; //The point cloud is initially unavailable

      //Initialize the downsampled point cloud with an empty pointer
      m_downsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
      downsampledPointCloudAvailable = false; //The downsampled point cloud is initially unavailable
  }


};
#endif
