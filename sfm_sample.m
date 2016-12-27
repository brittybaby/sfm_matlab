clc;
clear all;
close all;


% Read the two images

imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');
images = imageDatastore(imageDir);
I1 = readimage(images, 1);
I2 = readimage(images, 2);
figure
imshowpair(I1, I2, 'montage'); 
title('Original Images');


%Load the camera parameters// Camera calibration details
load upToScaleReconstructionCameraParameters.mat 


%  undistort the images using the distortion coefficients

I1 = undistortImage(I1, cameraParams);
I2 = undistortImage(I2, cameraParams);
figure 
imshowpair(I1, I2, 'montage');
title('Undistorted Images');



% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'MinQuality', 0.1);

% Visualize detected points
figure
imshow(I1, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);


% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);