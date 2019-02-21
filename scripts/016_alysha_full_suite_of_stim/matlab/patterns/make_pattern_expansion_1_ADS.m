% make_expansion_48.m
% PTW 2016/01/26 for Brad

FRAMERATE_FRAMESPERSEC = 60.0;
% arena:
ARENA_DEGPERPIX = 3.75;
NUMVERTPIX = 32;
ARENAINCLINATIONCENTER_PIX = 16.5;
NUMHORIZPIX = 96;
ARENAAZIMUTHCENTER_PIX = 44.5;
% constants:
DEGPERRAD = 180/pi;

MAXMOTIONDURATION_SEC = 2.5; %HALFSPEED (now quarterspeed)

%% Calculate arena coordinates
arenaCircumference_pix = 360.0/ARENA_DEGPERPIX;
arenaRadius_pix = arenaCircumference_pix/(2*pi);

arenaAzimuthCoordinates_deg = ([1:NUMHORIZPIX] - ARENAAZIMUTHCENTER_PIX)*ARENA_DEGPERPIX;
arenaAzimuthCoordinates_rad = arenaAzimuthCoordinates_deg/DEGPERRAD;
arenaInclinationCoordinates_rad = pi/2 - atan(([1:NUMVERTPIX] - ARENAINCLINATIONCENTER_PIX)/arenaRadius_pix);

EXPANSIONDURATION_SEC = 2.0;
EXPANSIONOFFSET_RAD = pi/4;

t_sec = [0:1/FRAMERATE_FRAMESPERSEC:EXPANSIONDURATION_SEC];
numFrames = length(t_sec);

NUMVERTPIX = 32;
NUMHORIZPIX = 96;

frames = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames);

SPEED_M_PER_SEC = 2.0;

THETA_START_DEG = 3.75;
THETA_END_DEG = 75;

thetaStartRad = THETA_START_DEG*(pi/180);
thetaEndRad = THETA_END_DEG*(pi/180);

[az,inc] = meshgrid(arenaAzimuthCoordinates_rad-EXPANSIONOFFSET_RAD,arenaInclinationCoordinates_rad-pi/2);

width = EXPANSIONDURATION_SEC*SPEED_M_PER_SEC/(1/tan(thetaStartRad)-1/tan(thetaEndRad));
distanceStart = width/tan(thetaStartRad);
thetasRad = atan(width./(distanceStart - SPEED_M_PER_SEC*t_sec));
for frameInd = 1:numFrames
    tThisFrame_sec = t_sec(frameInd);
    thetaThisFrame_rad = thetasRad(frameInd);
   

    inds = sqrt(az.^2 + inc.^2) <= thetaThisFrame_rad;
    %inds = imresize(inds,[NUMVERTPIX NUMHORIZPIX]); 
    frames(:,:,frameInd) = inds;
end

NUMVERTPIX = 32;
NUMHORIZPIX = 96;
%%
allPats = zeros(NUMVERTPIX, NUMHORIZPIX, numFrames, 1);
allPats(:,:,:,1) = 1 - frames;
%% put in language of panels
pattern.x_num = size(allPats,3);
pattern.y_num = size(allPats,4);
pattern.num_panels = 48; 	% This is the number of unique Panel IDs required.
pattern.row_compression = 0;
pattern.gs_val = 1;
pattern.Pats = allPats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];

%thisFullFileName =  mfilename('fullpath');
%[directory_name,thisFileName,thisFileExtension] = fileparts(thisFullFileName);
directory_name = 'C:\Users\scientist\Documents\adesouza';
%%
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = Make_pattern_vector(pattern);

str = [directory_name '\Pattern_expansion_1_ADS'];
save(str, 'pattern');