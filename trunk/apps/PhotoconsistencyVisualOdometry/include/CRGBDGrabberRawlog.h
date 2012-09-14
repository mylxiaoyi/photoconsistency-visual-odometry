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

#ifndef CRGBDGRABBER_RAWLOG
#define CRGBDGRABBER_RAWLOG

#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/slam/CRawlog.h>
#include <memory> // for std::auto_ptr<>

#include "CRGBDGrabber.h"

using namespace mrpt::slam;

/*!This class captures RGBD frames from rawlog sensor data using the MRPT library. It grabs the intensity image as well as its depth image. It also grabs the timestamp of each frame if provided in the rawlog.*/
class CRGBDGrabberRawlog : public CRGBDGrabber
{

private:
  mrpt::slam::CObservation3DRangeScanPtr currentObservationPtr;

  mrpt::utils::CFileGZInputStream*  dataset; //File stream to the rawlog dataset

  bool endGrabbing; //Bool variable to indicate that no more observations can be read from the rawlog

  uint64_t lastTimestamp;

public:
    /*!Creates a CRGBDGrabberRawlog instance that grabs RGBD frames from the specified rawlog file.*/
    CRGBDGrabberRawlog(const std::string &rawlog_file);

    /*!Initializes the grabber object*/
    inline void init(){};

    /*!Retains the current RGBD frame.*/
    void grab(CFrameRGBD*);

    /*!Stop grabing RGBD frames.*/
    inline void stop(){};
};
#endif
