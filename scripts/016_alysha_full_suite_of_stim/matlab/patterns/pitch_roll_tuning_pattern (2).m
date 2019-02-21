%make_rotating_starfields_48P
% This pattern is designed for panoramic behavioral arena, not e-phys arena
% PTW 5/30/2013
% NOTES: could make more efficient by fixing some for loops
%       could make the starfield repeat by concatenating xyz coordinates
%       with translated ones...
%modified 9/16/15 BHD to rotate bilaterally for all three directions
%modified 10/5/15 to translate bilaterally in lift, slip, and thrust
%modififed 2/5/16 to cycle through pitch/roll axes in 30 deg increments
%% Define constants
% universe of points:
DENSITYOFPOINTS_PERCUBICMETER = 20;
MAXSENSORYRADIUS_METERS = 2.0;
% motion:
YAWANGULARVELOCITY_RADPERSEC = 0;
PITCHANGULARVELOCITY_RADPERSEC = pi;
ROLLANGULARVELOCITY_RADPERSEC = 0;

MOTIONDURATION_SEC = 5; %changed from 2
FRAMERATE_FRAMESPERSEC = 60.0;
% arena:
ARENA_DEGPERPIX = 3.75;
NUMVERTPIX = 32;
ARENAINCLINATIONCENTER_PIX = 16.5;
NUMHORIZPIX = 96;
ARENAAZIMUTHCENTER_PIX = 44.5;
% constants:
DEGPERRAD = 180/pi;
NUM_DIRECTIONS = 12;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

%% Populate space with uniformly randomly distributed points
%speed_metersPerSec = sqrt(XVELOCITY_METERSPERSEC^2 + YVELOCITY_METERSPERSEC^2 + ZVELOCITY_METERSPERSEC^2);
%maxDistMeters = MAXSENSORYRADIUS_METERS + MOTIONDURATION_SEC*speed_metersPerSec;
maxDistMeters = MAXSENSORYRADIUS_METERS;
numPoints = round(DENSITYOFPOINTS_PERCUBICMETER*(2*maxDistMeters)^3);
xyz_meters = (rand(numPoints,3)*2 - 1)*maxDistMeters;

%% Calculate frames

t_sec = [0:1/FRAMERATE_FRAMESPERSEC:MOTIONDURATION_SEC - 1/FRAMERATE_FRAMESPERSEC];
numFrames = length(t_sec);

allPats = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames, NUM_DIRECTIONS);


frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    %xyzThisFrame_meters = xyz_meters - ones(numPoints,1)*[XVELOCITY_METERSPERSEC,YVELOCITY_METERSPERSEC,ZVELOCITY_METERSPERSEC]*tThisFrame_sec;
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - PITCHANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        %isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
    imshow(frames(:,:,frameInd))
    drawnow
end


allPats(:,:,:,1) = frames;

%% Calculate frames

YAWANGULARVELOCITY_RADPERSEC = 0;
PITCHANGULARVELOCITY_RADPERSEC = 0;
ROLLANGULARVELOCITY_RADPERSEC = -pi;

frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    %xyzThisFrame_meters = xyz_meters - ones(numPoints,1)*[XVELOCITY_METERSPERSEC,YVELOCITY_METERSPERSEC,ZVELOCITY_METERSPERSEC]*tThisFrame_sec;
    R = sqrt(sum(xyz_meters(:,[1,3]).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,1)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    xThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xThisFrame, xyz_meters(:,2), zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        %isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
    imshow(frames(:,:,frameInd))
    drawnow
end

allPats(:,:,:,2) = frames;

%% Calculate frames

YAWANGULARVELOCITY_RADPERSEC = 0;
PITCHANGULARVELOCITY_RADPERSEC = -pi;
ROLLANGULARVELOCITY_RADPERSEC = 0;

frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    %xyzThisFrame_meters = xyz_meters - ones(numPoints,1)*[XVELOCITY_METERSPERSEC,YVELOCITY_METERSPERSEC,ZVELOCITY_METERSPERSEC]*tThisFrame_sec;
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - PITCHANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        %isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
    imshow(frames(:,:,frameInd))
    drawnow
end


allPats(:,:,:,3) = frames;

%% Calculate frames
YAWANGULARVELOCITY_RADPERSEC = 0;
PITCHANGULARVELOCITY_RADPERSEC = 0;
ROLLANGULARVELOCITY_RADPERSEC = pi;

frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    %xyzThisFrame_meters = xyz_meters - ones(numPoints,1)*[XVELOCITY_METERSPERSEC,YVELOCITY_METERSPERSEC,ZVELOCITY_METERSPERSEC]*tThisFrame_sec;
    R = sqrt(sum(xyz_meters(:,[1,3]).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,1)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    xThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xThisFrame, xyz_meters(:,2), zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        %isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
    imshow(frames(:,:,frameInd))
    drawnow
end

allPats(:,:,:,4) = frames;


%% Calculate ROLL (about +30) frames
ROLL_OFFSET_PIX = 30/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5+ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end

allPats(:,:,:,5) = frames;
%% Calculate ROLL (about +60) frames
ROLL_OFFSET_PIX = 60/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5+ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end


allPats(:,:,:,6) = frames;
%% Calculate ROLL (about +120) frames
ROLL_OFFSET_PIX = 120/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5+ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end


allPats(:,:,:,7) = frames;
%% Calculate ROLL (about +150) frames
ROLL_OFFSET_PIX = 150/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5+ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end


allPats(:,:,:,8) = frames;
%% Calculate ROLL (about -30) frames
ROLL_OFFSET_PIX = 30/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5-ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end

allPats(:,:,:,9) = frames;
%% Calculate ROLL (about -60) frames
ROLL_OFFSET_PIX = 60/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5-ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end

allPats(:,:,:,10) = frames;
%% Calculate ROLL (about -120) frames
ROLL_OFFSET_PIX = 120/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5-ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end


allPats(:,:,:,11) = frames;
%% Calculate ROLL (about -150) frames
ROLL_OFFSET_PIX = 150/ARENA_DEGPERPIX;
ARENAAZIMUTHCENTER_PIX = 44.5-ROLL_OFFSET_PIX;

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaAzimuthCoordinates_rad = mod(arenaAzimuthCoordinates_rad + pi, 2*pi) - pi;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

ROLLANGULARVELOCITY_RADPERSEC = pi;
frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    R = sqrt(sum(xyz_meters(:,2:3).^2,2));
    elevation = atan2(xyz_meters(:,3),xyz_meters(:,2)) - ROLLANGULARVELOCITY_RADPERSEC*tThisFrame_sec;
    yThisFrame = R.*cos(elevation);
    zThisFrame = R.*sin(elevation);
    xyzThisFrame_meters = [xyz_meters(:,1), yThisFrame, zThisFrame];
    distanceThisFrame_meters =  sqrt(sum(xyzThisFrame_meters.^2,2));
    inclinationThisFrame_rad = acos(xyzThisFrame_meters(:,3)./distanceThisFrame_meters);
    azimuthThisFrame_rad = atan2(xyzThisFrame_meters(:,2),xyzThisFrame_meters(:,1));
    for pointIndex = 1:numPoints 
        distanceThisPoint_meters = distanceThisFrame_meters(pointIndex);
        inclinationThisPoint_rad = inclinationThisFrame_rad(pointIndex);
        azimuthThisPoint_rad = azimuthThisFrame_rad(pointIndex);
        isInSensoryRange = (distanceThisPoint_meters <= MAXSENSORYRADIUS_METERS);
        isInInclinationRange = ((inclinationThisPoint_rad >= min(arenaInclinationCoordinates_rad)) & (inclinationThisPoint_rad <= max(arenaInclinationCoordinates_rad)));
        isInAzimuthRange = ((azimuthThisPoint_rad >= min(arenaAzimuthCoordinates_rad)) & (azimuthThisPoint_rad <= max(arenaAzimuthCoordinates_rad)));
        if isInSensoryRange & isInInclinationRange & isInAzimuthRange %turn on pixel at this point
            [minE, inclinationCoordinate] = min(abs(inclinationThisPoint_rad - arenaInclinationCoordinates_rad));
            [minA, azimuthCoordinate] = min(abs(azimuthThisPoint_rad - arenaAzimuthCoordinates_rad));
            frames(inclinationCoordinate,azimuthCoordinate,frameInd) = 1;
        end
    end
     imshow(frames(:,:,frameInd))
     drawnow
end


allPats(:,:,:,12) = frames;

%%
allPats = -1*allPats+ 1; %invert light and dark
%% put in language of panels


pattern.x_num = numFrames;
pattern.y_num = size(allPats,4);
pattern.num_panels = 48; 	% This is the number of unique Panel IDs required.
pattern.row_compression = 0;
pattern.gs_val = 1;
pattern.Pats = allPats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];

%thisFullFileName =  mfilename('C:\MatlabRoot\iorodeopanels383974f4d24b\Matlab Codes\Patterns');
%[directory_name,thisFileName,thisFileExtension] = fileparts(thisFullFileName);
%directory_name = 'C:\Users\nebulosa\Documents\panels\Matlab Codes\Patterns\Peter';
%%
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = Make_pattern_vector(pattern);

directory_name = 'C:\MatlabRoot\iorodeopanels383974f4d24b\Matlab Codes\Patterns';
str = [directory_name '\Pattern_pitch_roll_48P_bd'];
save(str, 'pattern');



