/*
 *  Photoconsistency-Visual-Odometry
 *  Multiscale Photoconsistency Visual Odometry from RGBD Images
 *  Copyright (c) 2011-2012, Miguel Algaba Borrego
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

#ifndef POINTCLOUD_DOWNSAMPLER
#define POINTCLOUD_DOWNSAMPLER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*!This class implements a fast and straightforward algorithm to downsample organized 3D point clouds. The algorithm takes the median point of a square region (downsamplingStep x downsamplingStep) of 3D points to mitigate the data noise.*/
class PointCloudDownsampler
{
private:
  int downsamplingStep;
public:
  /*!Constructor of an instance of PointCloudDownsampler given the downsamplingStep*/
  PointCloudDownsampler(const int = 8);

  /*!Sets the desired downsamplingStep to the given value*/
  void setDownsamplingStep(const int);

  /*!Downsamples the point cloud given by pointCloudPtr skiping the RGB information. The resulting downsampled point cloud is returned in downsampledPointCloudPtr.*/
  void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledPointCloudPtr);

  /*!Downsamples the point cloud given by pointCloudPtr. The color information will be computed by taking the median RGB value in the square region. The resulting downsampled point cloud is returned in downsampledPointCloudPtr.*/
  void downsamplePointCloudColor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
};
#endif
