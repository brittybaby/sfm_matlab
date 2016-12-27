    % Extract MSER features
    I = imread('cameraman.tif');
    regions = detectMSERFeatures(I);
 
    % Use MSER with upright SURF feature descriptor
    [features, validPoints] = extractFeatures(I,regions,'Upright',true);
 
    % Visualize SURF features corresponding to the MSER ellipse centers 
    % along with scale and orientation.
    figure
    imshow(I)
    hold on
    plot(validPoints,'showOrientation',true)