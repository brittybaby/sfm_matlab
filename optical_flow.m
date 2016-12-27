clc;
clear all;
close all;


I1 = im2double(rgb2gray(imread('/home/nets/projects/Matlab/Stereo/sample_images_l/image1.jpg')));
I2 = im2double(rgb2gray(imread('/home/nets/projects/Matlab/Stereo/sample_images_l/image1.jpg')));
%I2 = im2double(rgb2gray(imread('/home/nets/projects/Matlab/Round_table/im4.bmp')));
opticFlow = opticalFlowHS;
flow = estimateFlow(opticFlow, I1); 
%opticalFlow = vision.OpticalFlow('ReferenceFrameSource', 'Input port')                                                          
%flow = step(opticalFlow, I1, I2);
figure;
imshow(I1);hold on
%plot(flow);
plot(flow, 'DecimationFactor', [5 5], 'ScaleFactor', 60)


