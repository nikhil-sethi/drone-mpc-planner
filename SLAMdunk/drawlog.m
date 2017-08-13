clear
close all
%% Import data from text file.
% Script for importing data from the following text file:
%
%    I:\share\mu-g\SLAMdunk\build\log.txt
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2017/08/13 16:35:20

%% Initialize variables.
filename = 'I:\share\mu-g\SLAMdunk\build\log.txt';
delimiter = ';';
startRow = 2;

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
%	column16: double (%f)
%   column17: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
%   column21: double (%f)
%	column22: double (%f)
%   column23: double (%f)
%	column24: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
valid = dataArray{:, 1};
posErrX = dataArray{:, 2};
posErrY = dataArray{:, 3};
posErrZ = dataArray{:, 4};
velX = dataArray{:, 5};
velY = dataArray{:, 6};
velZ = dataArray{:, 7};
hoverthrottle = dataArray{:, 8};
autoThrottle = dataArray{:, 9};
autoRoll = dataArray{:, 10};
autoPitch = dataArray{:, 11};
autoYaw = dataArray{:, 12};
joyThrottle = dataArray{:, 13};
joyRoll = dataArray{:, 14};
joyPitch = dataArray{:, 15};
joyYaw = dataArray{:, 16};
joySwitch = dataArray{:, 17};
throttleP = dataArray{:, 18};
throttleI = dataArray{:, 19};
throttleD = dataArray{:, 20};
dt = dataArray{:, 21};
dx = dataArray{:, 22};
dy = dataArray{:, 23};
dz = dataArray{:, 24};


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%%derive / calc some stuff:
A(:,1) = posErrY ; 
A(:,2) = 0;
A(:,3) = -(autoThrottle - 1580) / (1650 - 1580) - 2.5; %heuristically scaled!
A(:,4) = joySwitch ; 

subplot(3,1,1)
plot(A)
legend('PosErrY [m]','Pos setpoint','- scaled throttle', 'Stabilization ON')
subplot(3,1,2)
hold on
plot(velY)
legend('velocity Y')
subplot(3,1,3)
plot(hoverthrottle)
legend('hover throttle')