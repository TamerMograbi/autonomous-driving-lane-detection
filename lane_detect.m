
v=VideoReader('project_video.mp4');

outputVideo = VideoWriter(fullfile(pwd,'output'));
outputVideo.FrameRate = v.FrameRate;
open(outputVideo);
figure, hold on;
lastGoodPatch = [];
while hasFrame(v)
    b = readFrame(v); % read first frame
    deNoisedB = medfilt2(rgb2gray(b)); % denoise image
    
    edgesIm= edge(deNoisedB,'sobel'); % show edges
    
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
    lines = houghlines(edgesIm,theta,rho,P,'FillGap',5,'MinLength',7);
    
    
    patchCoords = [];
    addedNegTheta = false;
    addedPosTheta = false;
    max_len = 0;
 
    for k = 1:length(lines)
       if addedNegTheta == true && addedPosTheta == true
           break;
       end
       if (addedNegTheta == true && lines(k).theta < 0) || (addedPosTheta == true && lines(k).theta > 0)
           continue;
       end
       ray = lines(k).point2 - lines(k).point1;
       factor_dist = -10;
           
       %everything less than h/1.5 was blacked out.
       %here i found the x coord of the lines at height h/1.5
       topXcoord = (lines(k).rho - (h/1.5)*sind(lines(k).theta))/cosd(lines(k).theta);
       %find the x coord of the line at the bottom of the screen (y = h)
       bottomXCoord = (lines(k).rho - h*sind(lines(k).theta))/cosd(lines(k).theta);
       %extended_p2 = lines(k).point2+ray*factor_dist;
       %xy = [lines(k).point1; extended_p2];
       xy = [bottomXCoord, h; topXcoord,h/1.5 ];
       
       
       if lines(k).theta < 0
           patchCoords = [ patchCoords [topXcoord,bottomXCoord;(h/1.5),h]];
           %factor_dist = 10;
           addedNegTheta = true;
           color = 'green';
           
       else
           patchCoords = [ patchCoords [bottomXCoord,topXcoord;h,(h/1.5)]];
           addedPosTheta = true;
           color = 'red';
       end
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color',color);

       % Plot beginnings and ends of lines
       %plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       %plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

       % Determine the endpoints of the longest line segment
       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end
    %figure,imshow(b), hold on;
    
%     p1 = patch(patchCoords(1,:),patchCoords(2,:),'red')
%     p1.FaceVertexAlphaData = 0.2;    % Set constant transparency 
%     p1.FaceAlpha = 'flat' ;          % Interpolate to find face transparency
    
    pos_hexagon = [340 163 305 186 303 257 334 294 362 255 361 191];
    [m,n] = size(patchCoords);
    if m ~= 2 || n ~= 4
        %disp(lastGoodPatch);
        bWithShape = insertShape(b,'FilledPolygon',{lastGoodPatch},...
        'Color', {'green'},'Opacity',0.7);
        %imshow(bWithShape)
        writeVideo(outputVideo,bWithShape);
        continue
    end
    pos_patch = [patchCoords(1,1) patchCoords(2,1) patchCoords(1,2) patchCoords(2,2) patchCoords(1,3) patchCoords(2,3) patchCoords(1,4) patchCoords(2,4) ];
    lastGoodPatch = pos_patch;
    disp("last good patch");
    disp(lastGoodPatch);
    bWithShape = insertShape(b,'FilledPolygon',{pos_patch},...
    'Color', {'green'},'Opacity',0.7);
    writeVideo(outputVideo,bWithShape);
    %imshow(bWithShape)
    % highlight the longest line segment
   % plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');
     
    %imshow(edgesIm) % display image
%     bWithShape = insertShape(b,'FilledPolygon',{[patchCoords(1,1),patchCoords(2,1),patchCoords(2,1),patchCoords(2,2)]},...
%     'Color', {'white','red'},'Opacity',0.7);
%     imshow(bWithShape);
%     writeVideo(outputVideo,bWithShape)
end
close(outputVideo);

