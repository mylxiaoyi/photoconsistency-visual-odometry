close all;clear all;clc;

load build/errorFunction.txt;

errorFunction2D = reshape(errorFunction,640,480)';
%figure,imshow(uint8(abs(errorFunction2D)));
figure,imshow(abs(errorFunction2D));
title('Error function 2D');

imgDiff = imread('build/imgDiff.png');
figure,imshow(imgDiff);
title('imgDiff OpenCV');

squaredError = errorFunction'*errorFunction


