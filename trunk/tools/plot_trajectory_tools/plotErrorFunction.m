close all;clear all;clc;

load build/errorFunction.txt;

plot(errorFunction,'+');
axis([0 size(errorFunction,1) -200 200]);

squaredError = errorFunction'*errorFunction
