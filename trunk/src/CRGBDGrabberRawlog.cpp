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

#include "../include/CRGBDGrabberRawlog.h"

#include <iostream>

CRGBDGrabberRawlog::CRGBDGrabberRawlog(const std::string &rawlog_file = std::string())
{
    //Open the rawlog file
    dataset = new mrpt::utils::CFileGZInputStream();
    if (!dataset->open(rawlog_file))
        throw std::runtime_error("Couldn't open rawlog dataset file for input...");

    // Set external images directory:
    mrpt::utils::CImage::IMAGES_PATH_BASE = mrpt::slam::CRawlog::detectImagesDirectory(rawlog_file);

    endGrabbing = false;
    lastTimestamp = 0;
}

//CRGBDGrabberRawlog::~CRGBDGrabberRawlog(){}

void CRGBDGrabberRawlog::grab(CFrameRGBD* framePtr)
{
    bool grabbed=false;

    while(!grabbed)
    {
        //Read observations until we get a CObservation3DRangeScan
        mrpt::slam::CObservationPtr obs;
        do
        {
            try
            {
                *dataset >> obs;
            }
            catch (std::exception &e)
            {
                throw std::runtime_error( std::string("\nError reading from dataset file (EOF?):\n")+std::string(e.what()) );
            }
        } while (!IS_CLASS(obs,CObservation3DRangeScan));

        // We have one observation:
        currentObservationPtr = CObservation3DRangeScanPtr(obs);

        if(currentObservationPtr->timestamp>lastTimestamp) //discard frames grabbed before
        {
            currentObservationPtr->load(); // *Important* This is needed to load the range image if stored as a separate file.

            if (currentObservationPtr->hasRangeImage &&
                currentObservationPtr->hasIntensityImage)
            {
                //Retain the RGB image of the current frame
                cv::Mat rgbImage = cv::Mat((IplImage *) currentObservationPtr->intensityImage.getAs<IplImage>()).clone();
                framePtr->setRGBImage(rgbImage);

                //Retain the depth image of the current frame
                cv::Mat depthImage(rgbImage.rows,rgbImage.cols,CV_32FC1);
                mrpt::math::CMatrix depthImageMRPT = currentObservationPtr->rangeImage;

                #pragma omp parallel for
                for(int i=0;i<depthImage.rows;i++)
                {
                    for(int j=0;j<depthImage.cols;j++)
                    {
                        depthImage.at<float>(i,j)=depthImageMRPT(i,j);
                    }
                }
                framePtr->setDepthImage(depthImage);

                //Retain the timestamp of the current frame
                framePtr->setTimeStamp(currentObservationPtr->timestamp);
                grabbed=true;
            }
        }
    }
}
