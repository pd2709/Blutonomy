% csvToImage.m
close all
clear all

%addpath('csvData')

%filename = 'data00001.csv'
[fileName,pathName] = uigetfile({'*.csv'},'Select Oculus data file')
filePathName = [pathName fileName]


sonarData = importdata(filePathName);

figure()
imshow(sonarData, []);

fprintf("The min value of the csv file is : %d \n ", min(min(sonarData)))
fprintf("The max value of the csv file is : %d \n ", max(max(sonarData)))
fprintf("The rms value of the csv file is : %d \n ", mean(rms(sonarData)))

%% added by RV
dynamicRange = [-80 0];
sonarData = transpose(sonarData);
figure
imagesc(20*log10(sonarData / max(sonarData(:))),dynamicRange)
colorbar
xlabel('beams')
ylabel('range')
title({'Range-Azimuth' fileName})
ax = gca;
ax.YDir = 'normal';


% define the field-of-view of the sonar
imFov = 60; % 2.1MHz degrees / 1.2MHz 130 deg
% define the minimum and maximum range of the sonar
imRangeY = [0.2 1.0];
% calculate the cross-range distance
imRangeX = [-0.5 0.5] * imFov*pi/180 * max(imRangeY);

% the number of cross range cells is odd so that it is symmetrical about the centre beam
% the sonar only has even number of beams?
imOut = slice2sector(sonarData,min(imRangeY),imRangeY,257);

dynamicRange = [-80 0];
figure
imagesc(imRangeX,imRangeY,20*log10(imOut / max(imOut(:))),dynamicRange)
xlabel('Cross-Range (m)')
ylabel('range (m)')
title({'Range-Azimuth' fileName})
ax = gca;
ax.YDir = 'normal';
colormap(parula)
colorbar
axis square



% enter the beams in Matlab format eg 3 7 9, or 6:9
str = input('Enter Az beams to display (x:y) : ','s')
azBeams = eval(['[' str ']']);
figure
plot(sonarData(:,azBeams))
grid
xlabel('Range')
ylabel('Amplitude')
title({['Amplitude Az:' sprintf('%d ',azBeams)] fileName})


% check what unique values are used in the data
slice = sonarData(:);
slice = sort(slice);
slice = unique(slice);
figure;plot(slice,'x');grid

