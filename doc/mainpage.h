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

/*!\mainpage Documentation Overview
 *
 * \section intro_sec Introduction
 *
 * This is the documentation of the Photoconsistency-Visual-Odometry project. This project implements a method to estimate the rigid transformation that best aligns a pair of RGBD frames based on the maximization of a photoconsistency error function for visual odometry applications. To estimate the rigid transformation, the method performs a photoconsistency error function optimization at different image scales.
 * 
 * \image html  PhotoconsistencyOdometryMap.png "Top view of an indoor 3D map reconstructed using the estimated Visual Odometry as a first pose approximation and GICP for pose refinement."
 *
 * \section dependencies_sec Dependencies
 * This project uses several open-source libraries to build the whole solution. The main dependencies are:
 *    - Ceres Solver: http://code.google.com/p/ceres-solver/ 
 *    - OpenCV: http://opencv.willowgarage.com/wiki/
 *    - PCL: http://pointclouds.org/
 *    - MRPT: http://www.mrpt.org/
 *    - Eigen: http://eigen.tuxfamily.org
 *
 * \section install_sec Installation
 * This project has been implemented and tested in Ubuntu 11.04 and 11.10. To compile the source code you need to install the dependencies first. After that, follow the following steps to compile the project.
 *    - Substitute the ceres-solver/include/ceres/jet.h header by the one inside PhotoconsistencyVisualOdometry/third_party/jet.h.
 *
 *    - Generate the Code::Blocks project.
 *         -# Open CMake. 
 *         -# Set the source directory to PhotoconsistencyVisualOdometry and the build directory to PhotoconsistencyVisualOdometry/build.
 *         -# Set OpenCV_DIR and MRPT_DIR to the OpenCV and MRPT build directories respectively.
 *         -# Specify the Ceres include directory (ceres-solver/include) to the CERES_INCLUDE_DIRECTORY variable.
 *         -# Specify the Ceres library directory (ceres-solver_build/internal/ceres) to the CERES_LIB_DIRECTORY variable. 
 *         -# Configure.
 *         -# Generate.
 *
 *    - Compile the PhotoconsistencyVisualOdometry project.
 *         -# Open the PhotoconsistencyVisualOdometry/build/PhotoconsistencyVisualOdometry.cbp project.
 *         -# Compile.       
 *
 *    - [optional] Install CVPR tools dependencies and download the CVPR tools inside the PhotoconsistencyVisualOdometry/tools/rgbd_benchmark_tools directory.
\verbatim 
sudo apt-get install python-numpy
sudo apt-get install python-matplotlib 
cd PhotoconsistencyVisualOdometry/tools
svn co https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/
\endverbatim
 *
 *    - [optional] Download the file 'rigid_transform_3D.m' inside the PhotoconsistencyVisualOdometry/tools/plot_trajectory_tools directory. The file can be found in the following link: http://nghiaho.com/uploads/code/rigid_transform_3D.m
\verbatim 
cd PhotoconsistencyVisualOdometry/tools/plot_trajectory_tools
wget http://nghiaho.com/uploads/code/rigid_transform_3D.m
\endverbatim
 *	
 * \section usage_sec Software usage
 * 
 * After compiling the project, two executables should appear in the PhotoconsistencyVisualOdometry/build directory. The program PhotoconsistencyFrameAlignment estimates the 3DoF/6DoF rigid body motion between two RGBD frames loaded from file and shows the resulting difference image (Note: the 3DoF warping function is not yet implemented). The program PhotoconsistencyVisualOdometry estimates the trajectory of the Kinect sensor using a pairwise alignment approach. This program generates 3D point clouds transformed to the same reference frame to show the alignment results. The RGB-D data can be provided using a file interface that grabs RGB-D frames from .rawlog datasets using the MRPT library (TODO: in the following releases, an OpenNI interface will be added to allow online operation). In the following link there are some RGB-D datasets (.rawlog) that can be used to test the programs.
 *
 * http://www.mrpt.org/robotic_datasets
 * 
 * The original datasets can be found in the following link. However they are not prepared to be used in this sofware.
 *
 * http://cvpr.in.tum.de/data/datasets/rgbd-dataset
 *
 * These RGB-D datasets also contain a very precise ground-truth that can be very useful to evaluate different configurations. In the previous link, the CVPR group also provides a set of tools to measure the error between the estimated trajectory and the real trajectory of the ground-truth.	
 *
 * \subsection PhotoconsistencyFrameAlignment
 * This program estimates the 3DoF/6DoF rigid transformation between two RGBD frames loaded from file, maximizing the photoconsistency between the warped source intensity image and the target intensity image. To configure the optimization process, provide a configuration file to the algorithm like the ones in the PhotoconsistencyVisualOdometry/config_files directory.
 *
\verbatim 
cd PhotoconsistencyVisualOdometry/build
./PhotoconsistencyFrameAlignment <config_file.yml> <imgRGB0.png> <imgDepth0.png> <imgRGB1.png>
\endverbatim
 *
 *
 * \image html PhotoconsistencyDifferenceImage.png "OpenCV window that shows the resulting difference image from the photoconsistency maximization process."
 *
 *
 * \subsection PhotoconsistencyVisualOdometry
 * This program estimates the trajectory of the sensor and generates a 3D map from the provided secuence of RGB-D frames. The program stores the results in the PhotoconsistencyVisualOdometry/results directory. The program generates .pcd files representing each keyframe transformed to the original reference frame. These files are generated in the PhotoconsistencyVisualOdometry/results/pcd_files directory. The program can be compiled to use an Iterative Closest Point algorithm to refine the estimated rigid transformation. To refine the rigid transformation using ICP (recommended), set the ENABLE_ICP_POSE_REFINEMENT define to 1. Another useful feature is the possibility to save the estimated trajectory (and ground-truth if provided) to file. This is useful to visualize the estimated trajectory, as well as to compare it with the ground-truth using the CVPR tools. If you don't need to visualize or compare the estimated and ground-truth trajectories, simply set the ENABLE_SAVE_TRAJECTORY to 0.
 *
 * The rest of the configuration parameters can be passed to the program through the command line. The main parameters are the ground-truth file, to compare with the estimated trajectory (needs the CVPR tools) and the Iterative Closest Point method used to refine the pose.
 *
\verbatim 
cd PhotoconsistencyVisualOdometry/build
./PhotoconsistencyVisualOdometry <config_file.yml> <rawlog_file.rawlog> [options]
       options: 
               -g          Ground truth file <groundtruth.txt>
               -m          Iterative Closest Point method:
                           0: GICP
                           1: ICP-LM
                           2: ICP
               -d          Maximum correspondence distance (ICP/ICP-LM/GICP):
               -r          Ransac outlier rejection threshold (ICP/ICP-LM/GICP):
               -i          Maximum number of iterations (ICP/ICP-LM/GICP):
               -e          Transformation epsilon (ICP/ICP-LM/GICP):
               -s          Skip X frames. Process only one of X frames:
\endverbatim
 *
 * If you compile the program to save the trajectory, three files ("poses.mat", "trajectory.txt" and "estimated_trajectory.eps") will be saved to the PhotoconsistencyVisualOdometry/results directory: the first file contains the sequence of estimated poses (each row represents the 16 elements of a 4x4 rigid transformation matrix); the second file contains the same sequence of poses with timestamps, but using a 3D+Quaternion representation (to be used with the CVPR tools); the last file is a figure that shows the 2D top view of the sequence of poses (note: requires Octave to draw the trajectory). 
 *
 * Furthermore, if you provide a ground-truth file to the program, three more files ("groundtruth.txt", "trajectory.pdf" and "ground-truth_and_estimated_trajectory.eps") will be added to the PhotoconsistencyVisualOdometry/results directory: the first file is simply a copy of the provided ground-truth file; the second is the output file from the evaluate_ate.py CVPR tool, that shows the estimated trajectory and pose differences drawn over the ground-truth (note: requires the CVPR tools dependencies installed); the third file is a figure that shows the estimated trajectory over the ground-truth in the reference frame of the first RGBD frame (note: requires Octave and the file rigid_transform_3D.m).
 *
 * \subsection A Global map visualization
 * As said before, the PhotoconsistencyVisualOdometry program generates ".pcd" files inside the PhotoconsistencyVisualOdometry/results/pcd_files directory. These files contain the point cloud of each keyframe transformed to the same reference frame. To visualize the global map simply run the pcd_viewer tool from the PCL library with all or a subset of the generated ".pcd" files.
 *
\verbatim 
cd PhotoconsistencyVisualOdometry/build
../results/pcd_files/*.pcd 
\endverbatim
 *
 * \image html  PhotoconsistencyOdometry3DMap.png "PCL window that shows the generated global map of an indoor scene reconstructed using the Photoconsistency Visual Odometry as a first pose approximation and GICP for pose refinement. The results are stored in separate transformed_keyframe_x.pcd files that represent each keyframe point cloud transformed to the same reference frame; to open the .pcd files use the pcd_viewer tool of the PCL library."
 *
 * \subsection B CVPR tools
 * If you have the CVPR tools dependencies installed and have compiled the PhotoconsistencyVisualOdometry program to save the trajectories, the program will execute the evaluate_ate.py and evaluate_rpe.py tools that compute the absolute and relative error of the estimated trajectory compared to the ground-truth. Additionally, to visualize the goodness of the estimated trajectory, a ".pdf" file will be generated inside the PhotoconsistencyVisualOdometry/results directory, representing the ground-truth and the estimated trajectory seen from above. 
 *
 * \image html trajectory.svg "Estimated trajectory and pose differences drawn over the ground-truth using the CVPR tools."
 *
 * \subsection C Octave visualization scripts
 * Other useful tools for visualization are the "plot_odometry.m" and "plot_trajectory_groundtruth_estimated.m" Octave scripts: the first script loads the "poses.mat" file generated inside the PhotoconsistencyVisualOdometry/results (if compiled to save the trajectory) and plots all the estimated poses as seen from above; the second, plots the estimated trajectory and ground-truth as seen from above.
 *
 * \image html estimated_trajectory.svg "Estimated trajectory displayed using the plot_odometry.m script."
 *
 * \image html ground-truth_and_estimated_trajectory.svg "Estimated trajectory over the ground-truth using the Octave 'plot_trajectory_groundtruth_estimated.m' script."
 *
 * \author    Miguel Algaba Borrego
 * \n http://thecomputervision.blogspot.com/
 */
