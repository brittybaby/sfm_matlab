 % Extract corner features from an image.
    I = imread('cameraman.tif');
    corners = detectHarrisFeatures(I);
    [features, validCorners] = extractFeatures(I,corners);
 
    % Plot valid corner points on top of I
    figure
    imshow(I)
    hold on   
    plot(validCorners)