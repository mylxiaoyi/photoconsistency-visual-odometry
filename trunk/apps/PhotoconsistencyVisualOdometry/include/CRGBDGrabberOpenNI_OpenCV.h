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

#ifndef CRGBDGRABBER_OPENNI_OPENCV
#define CRGBDGRABBER_OPENNI_OPENCV

#include "opencv2/highgui/highgui.hpp" //OpenNI Kinect access
#include "opencv2/imgproc/imgproc.hpp"

#include "CRGBDGrabber.h"

/*!This class captures RGBD frames from an OpenNI compatible sensor using the OpenNI interface provided by the OpenCV library to access the sensor data. It grabs the intensity image as well as its corresponding depth image.*/

class CRGBDGrabberOpenNI_OpenCV : public CRGBDGrabber
{

private:

    cv::Mat currentRGBImg;
    cv::Mat currentDepthImg;
    cv::VideoCapture* capture;

public:
    /*!Creates a CRGBDGrabberOpenNI_OpenCV instance that grabs RGBD frames from an OpenNI compatible sensor.*/
    CRGBDGrabberOpenNI_OpenCV()
    {
        capture = new cv::VideoCapture( CV_CAP_OPENNI );
    }

    ~CRGBDGrabberOpenNI_OpenCV()
    {
        delete capture;
    }

    /*!Initializes the grabber object*/
    inline void init()
    {
        if( !capture->isOpened() )
        {
            std::cout << "Can not open a capture object." << std::endl;
        }

        capture->set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default
    };

    /*!Retains the current RGBD frame.*/
    void grab(CFrameRGBD*);

    /*!Stop grabing RGBD frames.*/
    inline void stop(){};
};
#endif
