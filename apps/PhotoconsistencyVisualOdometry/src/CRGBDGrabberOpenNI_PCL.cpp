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

#include "../include/CRGBDGrabberOpenNI_PCL.h"

#include <iostream>

boost::mutex mtx_;

void CRGBDGrabberOpenNI_PCL::rgb_cb_ (const boost::shared_ptr<openni_wrapper::Image>& img)
{
    //lock while we set the depth image;
    boost::mutex::scoped_lock lock (mtx_);
    currentBGRPtr = img;
}

void CRGBDGrabberOpenNI_PCL::depth_cb_ (const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
{
    //lock while we set the depth image;
    boost::mutex::scoped_lock lock (mtx_);
    currentDepthPtr = depth;
}

CRGBDGrabberOpenNI_PCL::CRGBDGrabberOpenNI_PCL()
{
     // Initialize the RGB and depth images. (Additionally initialize the auxiliary BGR image)
     currentRGBImg = cv::Mat(480,640,CV_8UC3);
     currentBGRImg = cv::Mat(480,640,CV_8UC3);
     currentDepthImg = cv::Mat(480,640,CV_32FC1);

     // Initialize the OpenNIGrabber
     interface = new pcl::OpenNIGrabber();

     // make callback functions from member functions
     boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&)> f = boost::bind (&CRGBDGrabberOpenNI_PCL::rgb_cb_, this, _1);
     boost::function<void(const boost::shared_ptr<openni_wrapper::DepthImage>&)> g = boost::bind (&CRGBDGrabberOpenNI_PCL::depth_cb_, this, _1);

     // connect callback function for desired signal.
     depth_connection = interface->registerCallback (g);
     rgb_connection = interface->registerCallback (f);
}

void CRGBDGrabberOpenNI_PCL::grab(CFrameRGBD* framePtr)
{
    {
        //lock while we set the RGB image;
        boost::mutex::scoped_lock lock (mtx_);

        //Grab the current RGB image
        currentBGRPtr->fillRGB(currentBGRImg.cols,currentBGRImg.rows,currentBGRImg.data,currentBGRImg.step);
    }
    cv::cvtColor(currentBGRImg,currentRGBImg,CV_RGB2BGR);
    framePtr->setRGBImage(currentRGBImg.clone());

    {
        //lock while we set the depth image;
        boost::mutex::scoped_lock lock (mtx_);

        //Grab the current depth image
        for(int r=0;r<currentDepthPtr->getHeight();r++)
        {
            for(int c=0;c<currentDepthPtr->getWidth();c++)
            {
                currentDepthImg.at<float>(r,c)=0.001*currentDepthPtr->getDepthMetaData()[r*currentDepthPtr->getWidth()+c];
            }
        }
    }
    framePtr->setDepthImage(currentDepthImg.clone());
}
