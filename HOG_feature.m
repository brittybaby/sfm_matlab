     clc;
     clear all;
     close all;
     
     I2 = imread('gantrycrane.png');
     corners   = detectFASTFeatures(rgb2gray(I2));
     strongest = selectStrongest(corners, 3);
     [hog2, validPoints, ptVis] = extractHOGFeatures(I2, strongest);
     figure;
     imshow(I2); hold on;
     plot(ptVis, 'Color','green');
  