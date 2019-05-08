clear;
v=VideoReader('project_video.mp4');

outputVideo = VideoWriter(fullfile(pwd,'output'));
outputVideo.FrameRate = v.FrameRate;
open(outputVideo);
lastGoodPatch = [];

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
           addedNegTheta = true;
       %line belongs to left lane
       else
           patchCoords = [ patchCoords [bottomXCoord,topXcoord;h,(h/1.5)]];
           addedPosTheta = true;
       end
    end

    [m,n] = size(patchCoords);
    %if for some reason couldn't find 2 lines for current frame
    %use last frame. (happens rarely)
    if m ~= 2 || n ~= 4
        bWithShape = insertShape(b,'FilledPolygon',{lastGoodPatch},...
        'Color', {'green'},'Opacity',0.7);
        writeVideo(outputVideo,bWithShape);
        continue;
    end
    pos_patch = [patchCoords(1,1) patchCoords(2,1) patchCoords(1,2) patchCoords(2,2) patchCoords(1,3) patchCoords(2,3) patchCoords(1,4) patchCoords(2,4) ];
    lastGoodPatch = pos_patch;
    bWithShape = insertShape(b,'FilledPolygon',{pos_patch},...
    'Color', {'green'},'Opacity',0.7);

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

