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

#define USE_PHOTOCONSISTENCY_ODOMETRY_ANALYTIC 1 // If set to 1 uses the CPhotoconsistencyOdometryAnalytic class
                                                 // else, uses the CPhotoconsistencyOdometryCeres class.
#define ENABLE_ICP_POSE_REFINEMENT 0
#define ENABLE_SAVE_TRAJECTORY 1
#define ENABLE_DISPLAY_ONLINE_MAP 0 // Enable this only for visualization/debug purposes

#include "include/CRGBDGrabberRawlog.h"
#include "include/CRGBDGrabberOpenNI_PCL.h"
#include "include/CFrameRGBD.h"
#if USE_PHOTOCONSISTENCY_ODOMETRY_ANALYTIC
    #include "phovo/include/CPhotoconsistencyOdometryAnalytic.h"
#else
    #include "phovo/include/CPhotoconsistencyOdometryCeres.h"
#endif
#include <pcl/io/pcd_io.h> //Save global map as PCD file
#include <pcl/common/transforms.h> //Transform the keyframe pointclouds to the original reference frame
#include <pcl/console/parse.h> //Parse the inputs of the program

#if ENABLE_ICP_POSE_REFINEMENT
    #include <pcl/registration/icp.h> //ICP
    #include <pcl/registration/icp_nl.h> //ICP LM
    #include <pcl/registration/gicp.h> //GICP
#endif

#if ENABLE_DISPLAY_ONLINE_MAP
#include <pcl/visualization/cloud_viewer.h>
#endif

#if ENABLE_SAVE_TRAJECTORY
#include <iomanip>
void addPoseToFile(Eigen::Matrix4f & pose3D,std::ofstream & posesFile)
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            posesFile << pose3D(i,j) <<" ";
        }
    }posesFile<<"\n";
}

#define TICKS_PER_MILISECOND 10000
#define EPOCH_DIFFERENCE_MILISECONDS 11644473600000LL
void add3DQuatPoseToFile(Eigen::Matrix4f & pose,
                         uint64_t timestamp,
                         std::ofstream & trajectoryFile)
{
    //Rotate the reference frame to the one needed by the CVPR tools
    //
    //          z  x             z'
    //          ^ ^             ^
    //          |/             /
    //    y <---     >>>>     --->x'
    //                        |
    //                        v
    //                        y'

    Eigen::Matrix4f T; T << 0,-1,0,0,
                            0,0,-1,0,
                            1,0,0,0,
                            0,0,0,1;

    Eigen::Matrix4f poseAux = pose*T.inverse();

    //Save pose timestamp [first convert FILETIME to Posix time (with miliseconds)]
    //convert from 100ns intervals to miliseconds and subtract number of miliseconds between epochs
    uint64_t timestamp_miliseconds = timestamp / TICKS_PER_MILISECOND - EPOCH_DIFFERENCE_MILISECONDS;
    trajectoryFile << timestamp_miliseconds/1000 << "." << std::setw(3) << std::setfill('0') << timestamp_miliseconds-(timestamp_miliseconds/1000)*1000 << " ";

    //Save the traslation and quaternion
    Eigen::Matrix3d rotMat;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rotMat(i,j)=poseAux(i,j);
            rotMat(i,j)=poseAux(i,j);
        }
    }
    Eigen::Quaterniond quat(rotMat);
    trajectoryFile << poseAux(0,3) << " " <<
                      poseAux(1,3) << " " <<
                      poseAux(2,3) << " " <<
                      quat.x() << " " <<
                      quat.y() << " " <<
                      quat.z() << " " <<
                      quat.w() << std::endl;
}
#endif

void saveKeyframeToFile(int keyframeIndx,CFrameRGBD & keyframe)
{
    std::stringstream keyframeFileName_ss;keyframeFileName_ss<<"../../../results/keyframe_"<<keyframeIndx;
    std::string keyframeFileName; keyframeFileName_ss>>keyframeFileName;
    std::cout<<"saving keyframe "<<keyframeIndx<<" to "<<keyframeFileName<<std::endl;
    keyframe.saveToFile(keyframeFileName);
}

void loadKeyframeFromFile(int keyframeIndx,CFrameRGBD & keyframe)
{
    std::stringstream keyframeFileName_ss;keyframeFileName_ss<<"../../../results/keyframe_"<<keyframeIndx;
    std::string keyframeFileName; keyframeFileName_ss>>keyframeFileName;
    std::cout<<"loading keyframe "<<keyframeIndx<<" from "<<keyframeFileName<<std::endl;
    keyframe.loadFromFile(keyframeFileName);
}

void printHelp()
{
    std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
    std::cout<<"./PhotoconsistencyVisualOdometry <config_file.yml> [options]"<<std::endl;
    std::cout<<"       options: "<<std::endl;
    std::cout<<"               -rawlog     RGBD rawlog file name <rgbd.rawlog>"<<std::endl;
    std::cout<<"               -g          Ground truth file <groundtruth.txt>"<<std::endl;
    #if ENABLE_ICP_POSE_REFINEMENT
    std::cout<<"               -m          Iterative Closest Point method:"<<std::endl;
    std::cout<<"                           0: GICP"<<std::endl;
    std::cout<<"                           1: ICP-LM"<<std::endl;
    std::cout<<"                           2: ICP"<<std::endl;
    std::cout<<"               -d          Maximum correspondence distance (ICP/ICP-LM/GICP):"<<std::endl;
    std::cout<<"               -r          Ransac outlier rejection threshold (ICP/ICP-LM/GICP):"<<std::endl;
    std::cout<<"               -i          Maximum number of iterations (ICP/ICP-LM/GICP):"<<std::endl;
    std::cout<<"               -e          Transformation epsilon (ICP/ICP-LM/GICP):"<<std::endl;
    #endif
    std::cout<<"               -s          Skip X frames. Process only one of X frames:"<<std::endl;
    std::cout<<"-----------------------------------------------------------------------------------"<<std::endl;
}

//Video sequence
int main(int argc, char **argv)
{
    if(argc<2){printHelp();return -1;}

    //Parse input arguments
    std::string rgbdFileName = "";
    pcl::console::parse_argument (argc, argv, "-rawlog", rgbdFileName);

    #if ENABLE_ICP_POSE_REFINEMENT
    int ICP_method = 0; //Desired ICP method [GICP: 0] [ICP-LM: 1] [ICP: 2]
    pcl::console::parse_argument(argc,argv,"-m",ICP_method);

    double dist = 0.1;  //Maximum correspondence distance
    pcl::console::parse_argument (argc, argv, "-d", dist);

    double rans = 0.05; //Ransac outlier rejection threshold
    pcl::console::parse_argument (argc, argv, "-r", rans);

    double transEpsilon = 1e-5;
    pcl::console::parse_argument (argc, argv, "-e", transEpsilon);

    int iter = 16; //Maximum number of iterations for ICP
    pcl::console::parse_argument (argc, argv, "-i", iter);
    #endif

    int skipXFrames = 0; //Number of frames to skip from the sequence
    pcl::console::parse_argument (argc, argv, "-s", skipXFrames);

    #if ENABLE_SAVE_TRAJECTORY
    std::string groundTruthFileName = "";
    pcl::console::parse_argument (argc, argv, "-g",groundTruthFileName);
    #endif

    //Set the camera parameters
    Eigen::Matrix3f cameraMatrix; cameraMatrix <<
                525., 0., 3.1950000000000000e+02,
                0., 525., 2.3950000000000000e+02,
                0., 0., 1.;

    //Define the global pose and a vector of global poses to save the trajectory of the sensor
    Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > globalPoses;
    std::vector<double> stateVector; stateVector.resize(6,0); //x,y,z,yaw,pitch,roll

    #if ENABLE_SAVE_TRAJECTORY
    std::vector<uint64_t> timestamps;
    std::ofstream posesFile; posesFile.open("../../../results/poses.mat"); //file to save the sequence of poses (as 4x4 matrices)
    std::ofstream trajectoryFile; trajectoryFile.open("../../../results/trajectory.txt"); //file to save the sequence of poses (as 3D translation + quaternions) with timestamp [to be used with CVPR tools]
    trajectoryFile << "# estimated trajectory\n# timestamp tx ty tz qx qy qz qw\n";
    #endif

    //Define the previous frame (frame 1) and current frame (frame 2)
    CFrameRGBD* frame1;
    CFrameRGBD* frame2;

    //Create and initialize an object to grab RGBD frames
    CRGBDGrabber * grabber;
    if(!rgbdFileName.empty())
    {
        grabber = new CRGBDGrabberRawlog(rgbdFileName);
    }
    else
    {
        grabber = new CRGBDGrabberOpenNI_PCL();
    }
    grabber->init();

    //Grab a RGB-D frame
    frame1 = new CFrameRGBD();
    grabber->grab(frame1);

    //Save first point cloud as a keyframe and add the first pose to the trajectory
    int keyframeIndx = 0;
    std::stringstream keyframeFileName_ss;keyframeFileName_ss<<keyframeIndx;std::string keyframeFileName; keyframeFileName_ss>>keyframeFileName;
    pcl::io::savePCDFile(std::string("../../../results/pcd_files/transformed_keyframe_").append(keyframeFileName).append(".pcd"),*frame1->getDownsampledPointCloud(cameraMatrix));
    keyframeIndx++;
    globalPoses.push_back(globalPose);
    #if ENABLE_SAVE_TRAJECTORY
    timestamps.push_back(frame1->getTimeStamp());
    #endif

    //Add first frame as a keyframe
    saveKeyframeToFile(0,*frame1);

    //Define the photoconsistency odometry object and set the input parameters
	#if USE_PHOTOCONSISTENCY_ODOMETRY_ANALYTIC
	PhotoconsistencyOdometry::Analytic::CPhotoconsistencyOdometryAnalytic photoconsistencyOdometry;
	#else
	PhotoconsistencyOdometry::Ceres::CPhotoconsistencyOdometryCeres photoconsistencyOdometry;
	#endif
	photoconsistencyOdometry.readConfigurationFile(std::string(argv[1]));
    photoconsistencyOdometry.setCameraMatrix(cameraMatrix);

    #if ENABLE_ICP_POSE_REFINEMENT
    //Select the ICP algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> *icp;
    if (ICP_method==2) //ICP
    {
        std::cout << "Using IterativeClosestPoint" << std::endl;
        icp = new pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>();
    }
    else if(ICP_method==1)//ICP LM
    {
        std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
        icp = new pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBA, pcl::PointXYZRGBA>();
    }
    else //GICP
    {
        std::cout << "Using GeneralizedIterativeClosestPoint" << std::endl;
        icp = new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>();
    }

    //Set the parameters for the ICP algorithm
    icp->setMaximumIterations (iter);
    icp->setMaxCorrespondenceDistance (dist);
    icp->setRANSACOutlierRejectionThreshold (rans);
    icp->setTransformationEpsilon(transEpsilon);
    #endif

    #if ENABLE_DISPLAY_ONLINE_MAP
    //Initialize the global map with the first frame
    pcl::visualization::CloudViewer viewer("3D map");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map;
    map.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    *map += *(frame1->getDownsampledPointCloud(cameraMatrix));
    #endif

    //Main pairwise alignment loop
    try{
    while(cv::waitKey(5)!='\n')
    {
        //Skip the specified number of frames
        for(int frameIndx=0;frameIndx<skipXFrames;frameIndx++)
        {
            frame2 = new CFrameRGBD();
            grabber->grab(frame2);
            delete frame2;
        }

        //Grab a RGB-D frame
        frame2 = new CFrameRGBD();
        grabber->grab(frame2);

        //Estimate the rigid transformation between frame1 and frame2
        Eigen::Matrix4f H;
        cv::TickMeter tm; tm.start();
        photoconsistencyOdometry.setSourceFrame(frame1->getIntensityImage(),frame1->getDepthImage());
        photoconsistencyOdometry.setTargetFrame(frame2->getIntensityImage());
        photoconsistencyOdometry.setInitialStateVector(stateVector); //initialize the current state vector with the one computed in the previous iteration
        photoconsistencyOdometry.optimize();
        photoconsistencyOdometry.getOptimalStateVector(stateVector);
        photoconsistencyOdometry.getOptimalRigidTransformationMatrix(H);
        tm.stop();
        std::cout<<"H (photoconsistency approximation):"<<std::endl<<H<<std::endl;
        std::cout << "Time = " << tm.getTimeSec() << " sec." << std::endl;

        //Show the difference image
        cv::Mat imgGray1Warped;
        PhotoconsistencyOdometry::warpImage<uint8_t>(frame1->getIntensityImage(),frame1->getDepthImage(),imgGray1Warped,H,cameraMatrix);
        cv::Mat imgDifference;
        cv::absdiff(frame2->getIntensityImage(),imgGray1Warped,imgDifference);
        cv::putText(imgDifference,"Press enter to stop grabbing frames",cv::Point(20,450),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);
        cv::imshow("imgDifference",imgDifference);

    	//Compute the pose increment from the 3D rigid body transformation
    	Eigen::Matrix4f T; T<<0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1;
    	Eigen::Matrix4f poseIncrement = T*H.inverse()*T.inverse();

        #if ENABLE_ICP_POSE_REFINEMENT
        //Perform ICP to refine the pose approximation
        tm.start();
        pcl::PointCloud<pcl::PointXYZRGBA> alignedCloud_not_used;
        icp->setInputCloud(frame1->getDownsampledPointCloud(cameraMatrix));
        icp->setInputTarget(frame2->getDownsampledPointCloud(cameraMatrix));
        icp->align(alignedCloud_not_used,poseIncrement.inverse());
        poseIncrement = icp->getFinalTransformation().inverse();
        H = poseIncrement.inverse();
        tm.stop();
        std::cout<<"H (ICP refinement):"<<std::endl<<H<<std::endl;
        std::cout << "Time = " << tm.getTimeSec() << " sec." << std::endl;
        #endif

        //Add a new keyframe
        bool keyframeCondition=false;
        static int count = 0;
        if (count>5){count=0;keyframeCondition=true;}else{count++;}
        if(keyframeCondition) //the frame1 is the new keyframe
        {
            //Update the global pose of the current keyframe
            globalPose = globalPose * poseIncrement;
            std::cout<<"Global pose: "<<std::endl<<globalPose<<std::endl;

                //Add the new estimated pose to the estimated trajectory
                globalPoses.push_back(globalPose);

                //Save the current keyframe to file
                saveKeyframeToFile(keyframeIndx,*frame2);
                keyframeIndx++;

            #if ENABLE_SAVE_TRAJECTORY
            timestamps.push_back(frame2->getTimeStamp());
            #endif

            #if ENABLE_DISPLAY_ONLINE_MAP
            //Transform the keyframe point cloud to the reference frame of the first keyframe cloud
            pcl::PointCloud<pcl::PointXYZRGBA> transformedCloud;
            pcl::transformPointCloud(*frame2->getDownsampledPointCloud(cameraMatrix),transformedCloud,globalPose);
            *map += transformedCloud;

            //Display the global map
            viewer.showCloud(map);
            #endif

            //Update the last keframe with the current frame
            delete frame1;
            frame1 = frame2;
        }
        else
        {
            //Delete the current frame
            delete frame2;
        }
    }

    //Stop the RGBD grabber
    grabber->stop();

    }catch(std::exception &e){} //try-catch to avoid crashing when there are no more frames in the dataset

    //Free the allocated objects
    delete grabber;

    //Transform each keyframe point cloud to the same reference frame using each global pose
    #pragma omp parallel for
    for(int keyframe_i_Indx=0;keyframe_i_Indx<globalPoses.size();keyframe_i_Indx++)
    {
        //Load the ith keyframe from file
        CFrameRGBD* keyframe_i = new CFrameRGBD();
        loadKeyframeFromFile(keyframe_i_Indx,*keyframe_i);

        //Transform the keyframe point cloud to the reference frame of the first keyframe cloud
        pcl::PointCloud<pcl::PointXYZRGBA> transformedCloud;
        pcl::transformPointCloud(*keyframe_i->getDownsampledPointCloud(cameraMatrix),transformedCloud,globalPoses[keyframe_i_Indx]);

        //Save the transformed keyframe cloud to file
        std::stringstream keyframeFileName_ss;keyframeFileName_ss<<keyframe_i_Indx;std::string keyframeFileName; keyframeFileName_ss>>keyframeFileName;
        pcl::io::savePCDFile(std::string("../../../results/pcd_files/transformed_keyframe_").append(keyframeFileName).append(".pcd"),transformedCloud);

        //Delete the ith keyframe memory
        delete keyframe_i;
    }

    #if ENABLE_SAVE_TRAJECTORY
    for(int keyframe_i_Indx=0;keyframe_i_Indx<globalPoses.size();keyframe_i_Indx++)
    {
        //Save the sequence of poses to file
        addPoseToFile(globalPoses[keyframe_i_Indx],posesFile);
        add3DQuatPoseToFile(globalPoses[keyframe_i_Indx],timestamps[keyframe_i_Indx],trajectoryFile);
    }

    //Close the file that contains the sequence of poses and the trajectory file
    posesFile.close();
    trajectoryFile.close();

    //Execute an Octave script to plot the estimated trajectory over the ground-truth
    std::string terminal_command = "octave ../../../tools/plot_trajectory_tools/plot_odometry.m";
    terminal_command.append(" ../../../results/poses.mat ");
    terminal_command.append(" '../../../results/estimated_trajectory.eps';");
    int plot_estimated_trajectories_exe = std::system(terminal_command.c_str());

    if(!groundTruthFileName.empty())
    {
        //Save the ground truth file to the results directory
        terminal_command ="cp ";
        terminal_command.append(groundTruthFileName);
        terminal_command.append(" ../../../results/groundtruth.txt;");
        int copy_groundtruth_exe = std::system(terminal_command.c_str());

        //Execute an Octave script to plot the estimated trajectory over the ground-truth
        terminal_command = "octave --eval 'addpath(\"../../../tools/plot_trajectory_tools/\")' ../../../tools/plot_trajectory_tools/plot_trajectory_groundtruth_estimated.m";
        terminal_command.append(" ../../../results/trajectory.txt ");
        terminal_command.append(groundTruthFileName).append(" '../../../results/ground-truth_and_estimated_trajectory.eps';");
        int plot_groundtruth_estimated_trajectories_exe = std::system(terminal_command.c_str());

        //Compute ATE and RPE from the ground-truth and estimated trajectories
        //python evaluate_ate.py --plot trajectory_odometry.png groundtruth.txt TrajectoryTimestamp.txt
        terminal_command = "python ../../../tools/rgbd_benchmark_tools/evaluate_ate.py --plot ../../../results/trajectory.pdf ";
        terminal_command.append(groundTruthFileName);
        terminal_command.append(" ../../../results/trajectory.txt;");
        int ate_exe = std::system(terminal_command.c_str());

        terminal_command = "python ../../../tools/rgbd_benchmark_tools/evaluate_rpe.py ";
        terminal_command.append(groundTruthFileName);
        terminal_command.append(" ../../../results/trajectory.txt;");
        int rpe_exe = std::system(terminal_command.c_str());
    }
    #endif

    return 0;
}
