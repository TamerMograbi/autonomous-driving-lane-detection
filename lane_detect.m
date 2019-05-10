clear;
v=VideoReader('project_video.mp4');

outputVideo = VideoWriter(fullfile(pwd,'output'));
outputVideo.FrameRate = v.FrameRate;
open(outputVideo);
lastGoodPatch = [];
lastCurveStr = '';
threshold = 10;

while hasFrame(v)
    b = readFrame(v); 
    hsvImage = rgb2hsv(b);
    yellowBinary = detectColor(hsvImage,'yellow');
    whiteBinary = detectColor(hsvImage,'white');
    binaryImg = yellowBinary | whiteBinary;
    deNoisedBinaryImg = medfilt2(binaryImg);

    edgesIm = edge(deNoisedBinaryImg,'sobel');
    
    [h,w,~] = size(edgesIm);
    for i = 1:(h/1.5)
       edgesIm(i, :) = zeros(1, w); % replace row i with zeroes
    end
    
    %zero out the upper part relative to the secondary diagonal.
    %starting from row 230+h
    for i = 1:h
        for j = 1:w
            if i+j < 230+h
                edgesIm(i, j) = 0; % replace col h with zeroes
            end
        end
    end
    
    %zero out elements above main diagonal
    for i = 1:h
        for j = 1:w
            if j-i > 400
               edgesIm(i, j) = 0;
            end
        end
    end
    
    % remove front of car
    for i = h-60:h
        for j = 1:w
            edgesIm(i, j) = 0;
        end
    end

    [H,theta,rho] = hough(edgesIm);
    P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(edgesIm,theta,rho,P,'FillGap',5,'MinLength',4);
    
    patchCoords = [];
    addedNegTheta = false;
    addedPosTheta = false;
    negThetaLine = lines(1); %random init
    posThetaLine = lines(1); %random init
    max_len = 0;

    for k = 1:length(lines)
        %if already found a line for right and left lane then stop
       if addedNegTheta == true && addedPosTheta == true
           break;
       end
       %don't add more than one line on the either the left or right lanes.
       if (addedNegTheta == true && lines(k).theta < 0) || (addedPosTheta == true && lines(k).theta > 0)
           continue;
       end
           
       %everything less than h/1.5 was blacked out.
       %here i found the x coord of the lines at height h/1.5
       topXcoord = (lines(k).rho - (h/1.5)*sind(lines(k).theta))/cosd(lines(k).theta);
       %find the x coord of the line at the bottom of the screen (y = h)
       bottomXCoord = (lines(k).rho - h*sind(lines(k).theta))/cosd(lines(k).theta);

       xy = [bottomXCoord, h; topXcoord,h/1.5 ];
       
       %line belongs to right lane
       if lines(k).theta < 0
           patchCoords = [ patchCoords [topXcoord,bottomXCoord;(h/1.5),h]];
           negThetaLine = lines(k);
           addedNegTheta = true;
       %line belongs to left lane
       else
           patchCoords = [ patchCoords [bottomXCoord,topXcoord;h,(h/1.5)]];
           posThetaLine = lines(k);
           addedPosTheta = true;
       end
    end

    [m,n] = size(patchCoords);
    %if for some reason couldn't find 2 lines for current frame
    %use last frame. (happens rarely)
    if m ~= 2 || n ~= 4
        bWithShape = insertShape(b,'FilledPolygon',{lastGoodPatch},...
        'Color', {'green'},'Opacity',0.7);
        %add curve text to image
        bWithShape = insertText(bWithShape,[30 30] ,lastCurveStr,'FontSize',18,'BoxColor',...
        'red','BoxOpacity',0.4,'TextColor','white');
        writeVideo(outputVideo,bWithShape);
        continue;
    end
    pos_patch = [patchCoords(1,1) patchCoords(2,1) patchCoords(1,2) patchCoords(2,2) patchCoords(1,3) patchCoords(2,3) patchCoords(1,4) patchCoords(2,4) ];
    lastGoodPatch = pos_patch;
    bWithShape = insertShape(b,'FilledPolygon',{pos_patch},...
    'Color', {'green'},'Opacity',0.7);
    
    %add curve text to image
    if addedPosTheta && addedNegTheta
        leftRightLineInters = getXIntersection(negThetaLine, posThetaLine);
        intersectionXcoord = leftRightLineInters(1);
        [bWithShape,lastCurveStr] = putCurveInfoInImage(intersectionXcoord,bWithShape,threshold,w);
    end

    writeVideo(outputVideo,bWithShape);
end
close(outputVideo);

%input: I - img in HSV color space. color - string, 'white' for white color
%any other string will be considered a yellow color
%output:binary image showing only the color color.
function  img = detectColor(I,color)

    % yellow color hsv ranges
    hueThresholdLow = 0.10;
    hueThresholdHigh = 0.14;
    saturationThresholdLow = 0.4;
    saturationThresholdHigh = 1;
    valueThresholdLow = 0.8;
    valueThresholdHigh = 1.0;
  if strcmp(color,'white') == true
% white color hsv ranges (took a bit of trial and error
        hueThresholdLow = 0.0;
        hueThresholdHigh = 1;
        saturationThresholdLow = 0;
        saturationThresholdHigh = 0.2;
        valueThresholdLow = 0.8;
        valueThresholdHigh = 1.0;
  end
 
    % returns a matrix where there are 1's at index i
    %if in H,S,V channels at index i, the values are in their correspondant
    %thresholds above
    mask = ( (I(:,:,1) >= hueThresholdLow) & (I(:,:,1) <= hueThresholdHigh) ) & ...
    ((I(:,:,2) >= saturationThresholdLow ) & (I(:,:,2) <= saturationThresholdHigh)) & ...
    ((I(:,:,3) >= valueThresholdLow ) & (I(:,:,3) <= valueThresholdHigh));


      img = mask;

end

% input: two lines in the form of r=xcos(theta)+ysin(theta)
% negLine is for right lane. posLine is for left lane
% output: interesection of both lines
function intersection = getXIntersection(negLine, posLine)    
    A = [cosd(negLine.theta),sind(negLine.theta) ; cosd(posLine.theta),sind(posLine.theta)];
    B = [negLine.rho;posLine.rho];
    intersection = linsolve(A,B);
    
end

%input: x coord intersection of left & right lanes, image, threshold, width
%w
%output: [image with curve info text added to it,the text added]
function [imageWithCurve,current_curve_str] = putCurveInfoInImage(intersectionXcoord,img,threshold,w)
        curve_str = 'none';
        if intersectionXcoord > threshold + w/2
            curve_str = 'right';
        end
        if intersectionXcoord < w/2 - threshold
            curve_str = 'left';
        end
        text_str = [ 'lanes intersect ' num2str(intersectionXcoord - w/2,'%0.2f') ' units away from center. curve=' curve_str];
        current_curve_str = text_str;
        imageWithCurve = insertText(img,[30 30] ,text_str,'FontSize',18,'BoxColor',...
        'red','BoxOpacity',0.4,'TextColor','white');

end

