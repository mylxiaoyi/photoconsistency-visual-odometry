# remove runtime generated files
cd results;
rm keyframe_*.bin;
rm poses.mat;
rm trajectory.txt;
rm estimated_trajectory.eps;
rm groundtruth.txt;
rm ground-truth_and_estimated_trajectory.eps;
rm trajectory.pdf;
cd pcd_files;
rm *.pcd;
cd ..;
cd ..;

