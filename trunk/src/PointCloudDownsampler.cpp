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

#include "../include/PointCloudDownsampler.h"

PointCloudDownsampler::PointCloudDownsampler(const int step)
{
    downsamplingStep = step;
}

//PointCloudDownsampler::~PointCloudDownsampler(){}

void PointCloudDownsampler::setDownsamplingStep(const int step)
{
    downsamplingStep=step;
}

void PointCloudDownsampler::downsamplePointCloud(
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledPointCloudPtr)
{
    int j;j=0;
    int width = pointCloudPtr->width;
    int height = pointCloudPtr->height;

    downsampledPointCloudPtr->points.resize(width*height/downsamplingStep*downsamplingStep);
    #pragma omp parallel for
    for(int r=0;r<height;r=r+downsamplingStep)
    {
        std::vector<double> xV;
        std::vector<double> yV;
        std::vector<double> zV;

        for(int c=0;c<width;c=c+downsamplingStep)
        {
            int nPoints=0;
            xV.resize(downsamplingStep*downsamplingStep);
            yV.resize(downsamplingStep*downsamplingStep);
            zV.resize(downsamplingStep*downsamplingStep);

            for(int r2=r;r2<r+downsamplingStep;r2++)
            {
                for(int c2=c;c2<c+downsamplingStep;c2++)
                {
                    //Check if the point has valid data
                    if(pcl_isfinite (pointCloudPtr->points[r2*width+c2].x) &&
                       pcl_isfinite (pointCloudPtr->points[r2*width+c2].y) &&
                       pcl_isfinite (pointCloudPtr->points[r2*width+c2].z) &&
                       0.3<pointCloudPtr->points[r2*width+c2].x &&
                       pointCloudPtr->points[r2*width+c2].x<5)
                    {
                        //Create a vector with the x, y and z coordinates of the square region
                        xV[nPoints]=pointCloudPtr->points[r2*width+c2].x;
                        yV[nPoints]=pointCloudPtr->points[r2*width+c2].y;
                        zV[nPoints]=pointCloudPtr->points[r2*width+c2].z;

                        nPoints++;
                    }
                }
            }

            //Check if there are points in the region
            if(nPoints>0)
            {
                xV.resize(nPoints);
                yV.resize(nPoints);
                zV.resize(nPoints);

                //Compute the median 3D point
                std::sort(xV.begin(),xV.end());
                std::sort(yV.begin(),yV.end());
                std::sort(zV.begin(),zV.end());

                pcl::PointXYZRGBA point;
                point.x=xV[nPoints/2];
                point.y=yV[nPoints/2];
                point.z=zV[nPoints/2];

                //Set the median point as the representative point of the region
                #pragma omp critical
                {
                    downsampledPointCloudPtr->points[j]=point;
                    j++;
                }
            }
        }
    }
    downsampledPointCloudPtr->points.resize(j);
    downsampledPointCloudPtr->width=downsampledPointCloudPtr->size();
    downsampledPointCloudPtr->height=1;
}

void PointCloudDownsampler::downsamplePointCloudColor(
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledPointCloudPtr)
{
    int j;j=0;
    int width = pointCloudPtr->width;
    int height = pointCloudPtr->height;

    downsampledPointCloudPtr->points.resize(width*height/downsamplingStep*downsamplingStep);
    #pragma omp parallel for
    for(int r=0;r<height;r=r+downsamplingStep)
    {
        std::vector<double> xV;
        std::vector<double> yV;
        std::vector<double> zV;
        std::vector<double> rV;
        std::vector<double> gV;
        std::vector<double> bV;

        for(int c=0;c<width;c=c+downsamplingStep)
        {
            int nPoints=0;
            xV.resize(downsamplingStep*downsamplingStep);
            yV.resize(downsamplingStep*downsamplingStep);
            zV.resize(downsamplingStep*downsamplingStep);
            rV.resize(downsamplingStep*downsamplingStep);
            gV.resize(downsamplingStep*downsamplingStep);
            bV.resize(downsamplingStep*downsamplingStep);

            for(int r2=r;r2<r+downsamplingStep;r2++)
            {
                for(int c2=c;c2<c+downsamplingStep;c2++)
                {
                    //Check if the point has valid data
                    if(pcl_isfinite (pointCloudPtr->points[r2*width+c2].x) &&
                       pcl_isfinite (pointCloudPtr->points[r2*width+c2].y) &&
                       pcl_isfinite (pointCloudPtr->points[r2*width+c2].z) &&
                       0.3<pointCloudPtr->points[r2*width+c2].x &&
                       pointCloudPtr->points[r2*width+c2].x<5)
                    {
                        //Create a vector with the x, y and z coordinates of the square region and RGB info
                        xV[nPoints]=pointCloudPtr->points[r2*width+c2].x;
                        yV[nPoints]=pointCloudPtr->points[r2*width+c2].y;
                        zV[nPoints]=pointCloudPtr->points[r2*width+c2].z;
                        rV[nPoints]=pointCloudPtr->points[r2*width+c2].r;
                        gV[nPoints]=pointCloudPtr->points[r2*width+c2].g;
                        bV[nPoints]=pointCloudPtr->points[r2*width+c2].b;

                        nPoints++;
                    }
                }
            }

            if(nPoints>0)
            {
                xV.resize(nPoints);
                yV.resize(nPoints);
                zV.resize(nPoints);
                rV.resize(nPoints);
                gV.resize(nPoints);
                bV.resize(nPoints);

                //Compute the median 3D point and median RGB value
                std::sort(xV.begin(),xV.end());
                std::sort(yV.begin(),yV.end());
                std::sort(zV.begin(),zV.end());
                std::sort(rV.begin(),rV.end());
                std::sort(gV.begin(),gV.end());
                std::sort(bV.begin(),bV.end());

                pcl::PointXYZRGBA point;
                point.x=xV[nPoints/2];
                point.y=yV[nPoints/2];
                point.z=zV[nPoints/2];
                point.r=rV[nPoints/2];
                point.g=gV[nPoints/2];
                point.b=bV[nPoints/2];

                //Set the median point as the representative point of the region
                #pragma omp critical
                {
                    downsampledPointCloudPtr->points[j]=point;
                    j++;
                }
            }
        }
    }
    downsampledPointCloudPtr->points.resize(j);
    downsampledPointCloudPtr->width=downsampledPointCloudPtr->size();
    downsampledPointCloudPtr->height=1;
}
