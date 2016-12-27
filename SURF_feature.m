    % Extract SURF features
    I = imread('cameraman.tif');
    points = detectSURFFeatures(I);
 
    [features, validPoints] = extractFeatures(I,points);
 
    % Visualize 10 strongest SURF features, including their scale, and 
    % orientation which was determined during the descriptor extraction
    % process.
    tenStrongestPoints = selectStrongest(validPoints,20);
    figure
    imshow(I)
    hold on
    plot(tenStrongestPoints,'showOrientation',true)