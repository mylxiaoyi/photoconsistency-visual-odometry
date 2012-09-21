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

#ifndef CRGBDGRABBER_OPENNI_PCL
#define CRGBDGRABBER_OPENNI_PCL

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>

#include "CRGBDGrabber.h"

/*!This class captures RGBD frames from an OpenNI compatible sensor using the OpenNI interface provided by the PCL library to access the sensor data. It grabs the intensity image as well as its corresponding depth image.*/

class CRGBDGrabberOpenNI_PCL : public CRGBDGrabber
{

private:

    cv::Mat currentRGBImg;
    cv::Mat currentBGRImg;
    cv::Mat currentDepthImg;
    pcl::OpenNIGrabber* interface;

    void rgb_cb_ (const boost::shared_ptr<openni_wrapper::Image>& img);
    void depth_cb_ (const boost::shared_ptr<openni_wrapper::DepthImage>& depth);

    boost::shared_ptr<openni_wrapper::Image> currentBGRPtr;
    boost::shared_ptr<openni_wrapper::DepthImage> currentDepthPtr;

    boost::signals2::connection rgb_connection;
    boost::signals2::connection depth_connection;

public:
    /*!Creates a CRGBDGrabberOpenNI_PCL instance that grabs RGBD frames from an OpenNI compatible sensor.*/
    CRGBDGrabberOpenNI_PCL();

    ~CRGBDGrabberOpenNI_PCL()
    {
        delete interface;
    }

    /*!Initializes the grabber object*/
    inline void init()
    {
        // start receiving point clouds
     	interface->start ();
        sleep(2);
    };

    /*!Retains the current RGBD frame.*/
    void grab(CFrameRGBD*);

    /*!Stop grabing RGBD frames.*/
    inline void stop()
    {
        depth_connection.disconnect();
        rgb_connection.disconnect();
        interface->stop();
    };
};
#endif
