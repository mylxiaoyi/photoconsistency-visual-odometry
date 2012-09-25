####################################
# Photoconsistency-Visual-Odometry #
####################################
Multiscale Photoconsistency Visual Odometry from RGBD Images
http://code.google.com/p/photoconsistency-visual-odometry/

1. Introduction:
----------------
The Phovo library (Photoconsistency-Visual-Odometry) implements methods to estimate Visual Odometry based on a photoconsistency error function optimization approach at different image scales. Photoconsistency-Visual-Odometry is licensed under BSD license, please see LICENSE.txt for a full text of the license. 

The project is divided into two parts: the Phovo library, which implements methods to align pairs of RGBD frames; and applications which demonstrate the usage of the Phovo library. The source code for the Phovo library is inside the "phovo" directory, while the code for the applications lie under the "apps" directory. 

2. Dependencies:
----------------
This project uses several open-source libraries to build the whole solution. The main dependencies for the Phovo library are:

    - OpenCV: http://opencv.willowgarage.com/wiki/
    - Eigen: http://eigen.tuxfamily.org  
    - OpenMP: http://openmp.org/wp/
    - [optional] Ceres Solver: http://code.google.com/p/ceres-solver/

Additionally, the applications depends on two more open-source libraries:

    - PCL: http://pointclouds.org/
    - MRPT: http://www.mrpt.org/

This project includes code developed by the authors of the Libmv library to propagate the image derivatives, needed to integrate the numeric image gradients with the auto-diff framework of Ceres Solver. Please see sample.h inside the third_party directory for their copyright notice. 
    
3. Installation:
----------------
This project has been implemented and tested in Ubuntu 11.04 and 11.10. To compile the source code you need to install the dependencies first. After that, follow the next steps to compile the project.

    - Generate the project for your IDE.
	- Open CMake.
        - Set the source directory to photoconsistency-visual-odometry/trunk and the build directory to photoconsistency-visual-odometry/trunk/build.
	- [optional] Compile Phovo library with Ceres-Solver:
		- Substitute the ceres-solver/include/ceres/jet.h header by the one inside photoconsistency-visual-odometry/trunk/third_party/jet.h.
		- Specify the Ceres include directory (ceres-solver/include) to the CERES_INCLUDE_DIRECTORY variable.
        	- Specify the Ceres library directory (ceres-solver_build/internal/ceres) to the CERES_LIB_DIRECTORY variable.
	- [optional] Enable or disable the BUILD_PHOTOCONSISTENCY_FRAME_ALIGNMENT_APPLICATION variable to build or not the PhotoconsistencyFrameAlignment application (default true).
	- [optional] Enable or disable the BUILD_PHOTOCONSISTENCY_VISUAL_ODOMETRY_APPLICATION variable to build or not the PhotoconsistencyVisualOdometry application (default false). Notice that the PhotoconsistencyVisualOdometry application depends on PCL, MRPT and Boost-serialization to compile.
        - Configure.
        - Generate.
	- Compile.

    - [optional] Install CVPR tools dependencies and download the CVPR tools inside the photoconsistency-visual-odometry/trunk/tools/rgbd_benchmark_tools directory.

      sudo apt-get install python-numpy
      sudo apt-get install python-matplotlib 
      cd photoconsistency-visual-odometry/trunk/tools
      svn co https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/

    - [optional] Download the file 'rigid_transform_3D.m' inside the photoconsistency-visual-odometry/trunk/tools/plot_trajectory_tools directory. The file can be found in the following link: http://nghiaho.com/uploads/code/rigid_transform_3D.m

      cd photoconsistency-visual-odometry/trunk/tools/plot_trajectory_tools
      wget http://nghiaho.com/uploads/code/rigid_transform_3D.m

4. Software usage:
------------------
After compiling the project, the Phovo binary library will be placed inside the photoconsistency-visual-odometry/trunk/build/phovo directory. If you set the BUILD_PHOTOCONSISTENCY_FRAME_ALIGNMENT_APPLICATION variable to true, the executable for the PhotoconsistencyFrameAlignment application will lie under the photoconsistency-visual-odometry/trunk/build/apps/PhotoconsistencyFrameAlignment directory. Additionally, if you configure and generate the project setting the BUILD_PHOTOCONSISTENCY_VISUAL_ODOMETRY_APPLICATION to true, the executable for the PhotoconsistencyVisualOdometry will be built inside the photoconsistency-visual-odometry/trunk/build/apps/PhotoconsistencyVisualOdometry directory. 

The program PhotoconsistencyFrameAlignment estimates the 3DoF/6DoF rigid body motion between two RGBD frames loaded from file and shows the resulting difference image (Note: the 3DoF warping function is not yet implemented). The program PhotoconsistencyVisualOdometry estimates the trajectory of the Kinect sensor using a pairwise alignment approach. This program generates 3D point clouds transformed to the same reference frame to show the alignment results. The RGB-D data can be provided accessing directly to the Kinect sensor data through the OpenCV-OpenNI interface or PCL-OpenNI interface, or even using a file interface that grabs RGB-D frames from .rawlog datasets using the MRPT library. In the following link there are some RGB-D datasets (.rawlog) that can be used to test the programs.

http://www.mrpt.org/robotic_datasets

The original datasets can be found in the following link. However they are not prepared to be used in this sofware.

http://cvpr.in.tum.de/data/datasets/rgbd-dataset

These RGB-D datasets also contain a very precise ground-truth that can be very useful to evaluate different configurations. In the previous link, the CVPR group also provides a set of tools to measure the error between the estimated trajectory and the real trajectory of the ground-truth.

4.1 PhotoconsistencyFrameAlignment:
-----------------------------------
This program estimates the 3DoF/6DoF rigid transformation between two RGBD frames loaded from file, maximizing the photoconsistency between the warped source intensity image and the target intensity image. To configure the optimization process, provide a configuration file to the algorithm like the ones inside the photoconsistency-visual-odometry/trunk/config_files directory.

cd photoconsistency-visual-odometry/trunk/build/apps/PhotoconsistencyFrameAlignment
./PhotoconsistencyFrameAlignment <config_file.yml> <imgRGB0.png> <imgDepth0.png> <imgRGB1.png> <imgDepth1.png>

Currently there are implemented three different classes to estimate the rigid transformation: CPhotoconsistencyOdometryAnalytic, CPhotoconsistencyOdometryBiObjective and CPhotoconsistencyOdometryCeres. To change between them, simply set the propper value to the USE_PHOTOCONSISTENCY_ODOMETRY_METHOD preprocessor variable.

#define USE_PHOTOCONSISTENCY_ODOMETRY_METHOD 0 // CPhotoconsistencyOdometryAnalytic: 0
                                               // CPhotoconsistencyOdometryCeres: 1
                                               // CPhotoconsistencyOdometryBiObjective: 2

4.2 PhotoconsistencyVisualOdometry:
-----------------------------------
This program estimates the trajectory of the sensor and generates a 3D map from the provided secuence of RGB-D frames. The program stores the results in the photoconsistency-visual-odometry/trunk/results directory. The program generates .pcd files representing each keyframe transformed to the original reference frame. These files are generated in the photoconsistency-visual-odometry/trunk/results/pcd_files directory. The program can be compiled to use an Iterative Closest Point algorithm to refine the estimated rigid transformation. To refine the rigid transformation using ICP, set the ENABLE_ICP_POSE_REFINEMENT preprocessor variable to 1. Another useful feature is the possibility to save the estimated trajectory (and ground-truth if provided) to file. This is useful to visualize the estimated trajectory, as well as to compare it with the ground-truth using the CVPR tools. If you don't need to visualize or compare the estimated and ground-truth trajectories, simply set the ENABLE_SAVE_TRAJECTORY variable to 0. Furthermore, it could be useful to visualize the resulting 3D map while it is being reconstructed; to enable the 3D map visualization, set the ENABLE_DISPLAY_ONLINE_MAP variable to 1.

The rest of the configuration parameters can be passed to the program through the command line. The main parameters are the ground-truth file, to compare with the estimated trajectory (needs the CVPR tools) and the Iterative Closest Point method used to refine the pose.

cd photoconsistency-visual-odometry/trunk/build/apps/PhotoconsistencyVisualOdometry
./PhotoconsistencyVisualOdometry <config_file.yml> [options]
       options: 
               -rawlog     RGBD rawlog file name <rgbd.rawlog>
               -interface  OpenNI interface:
                           0: OpenCV
                           1: PCL
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

If you compile the program to save the trajectory, three files ("poses.mat", "trajectory.txt" and "estimated_trajectory.eps") will be saved to the photoconsistency-visual-odometry/trunk/results directory: the first file contains the sequence of estimated poses (each row represents the 16 elements of a 4x4 rigid transformation matrix); the second file contains the same sequence of poses with timestamps, but using a 3D+Quaternion representation (to be used with the CVPR tools); the last file is a figure that shows the 2D top view of the sequence of poses (note: requires Octave to draw the trajectory).

Furthermore, if you provide a ground-truth file to the program, three more files ("groundtruth.txt", "trajectory.pdf" and "ground-truth_and_estimated_trajectory.eps") will be added to the photoconsistency-visual-odometry/trunk/results directory: the first file is simply a copy of the provided ground-truth file; the second is the output file from the evaluate_ate.py CVPR tool, that shows the estimated trajectory and pose differences drawn over the ground-truth (note: requires the CVPR tools dependencies installed); the third file is a figure that shows the estimated trajectory over the ground-truth in the reference frame of the first RGBD frame (note: requires Octave and the file rigid_transform_3D.m).

4.3 Global map visualization:
-----------------------------
As said before, the PhotoconsistencyVisualOdometry program generates ".pcd" files inside the photoconsistency-visual-odometry/trunk/results/pcd_files directory. These files contain the point cloud of each keyframe transformed to the same reference frame. To visualize the global map simply run the pcd_viewer tool from the PCL library with all or a subset of the generated ".pcd" files.

cd photoconsistency-visual-odometry/trunk/results/pcd_files
pcd_viewer *.pcd 

4.4 CVPR tools:
---------------
If you have the CVPR tools dependencies installed and have compiled the PhotoconsistencyVisualOdometry program to save the trajectories, the program will execute the evaluate_ate.py and evaluate_rpe.py tools that compute the absolute and relative error of the estimated trajectory compared to the ground-truth. Additionally, to visualize the goodness of the estimated trajectory, a ".pdf" file will be generated inside the photoconsistency-visual-odometry/trunk/results directory, representing the ground-truth and the estimated trajectory seen from above.

4.5 Octave visualization scripts:
---------------------------------
Other useful tools for visualization are the "plot_odometry.m" and "plot_trajectory_groundtruth_estimated.m" Octave scripts: the first script loads the "poses.mat" file generated inside the photoconsistency-visual-odometry/trunk/results (if compiled to save the trajectory) and plots all the estimated poses as seen from above; the second, plots the estimated trajectory and ground-truth as seen from above.

Contact information:
--------------------
miguel.algaba.borrego@gmail.com
