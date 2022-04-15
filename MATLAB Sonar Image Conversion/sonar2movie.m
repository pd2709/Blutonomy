function sonar2movie(pathName,displayRangeY,cfg)

% sonar2movie.m
% usage:
%   sonar2movie()
%
%   pathName = 'Q:\ENG\Technical\underwater communications\Blutonomy\Tests & Trials\Daniel's Pool with Oculus MBS 16112021\Test Data'
%\\rmstna01.au.thalesgroup.local\datatech\Shared\ENG\Technical\underwater communications\Blutonomy\Tests & Trials\Daniel''s Pool with Oculus MBS 16112021\Test Data\'
%   cfg.DR = [0 1];  cfg.scaling=1; cfg.axisSquare=1; cfg.map=copper; cfg.swapBytes = 1; 
%   cfg.DR = [-60 0];  cfg.scaling=2; cfg.axisSquare=1; cfg.map=copper; cfg.swapBytes = 1; 
%   cfg.DR = dynamic range , values are in dB [-60 0] or linear eg [0 1]
%   cfg.scaling = 1(linear) 2(dB)
%   cfg.axisSquare=1 (square pixels)
%   cfg.map=copper  (colormap, see Matlab function, common options are: parula, hot, jet,turbo 
%   cfg.swapBytes = 1 (use this to swap bytes, needed for data sets collected in 2021)
%
%   sonar2movie(pathName,[],cfg);

if nargin < 2
  cfg.DR = 80;
  cfg.map = gray;
end

if nargin < 1
  displayRangeY = [];
  cfg.DR = 80;
  cfg.map = gray;
end

[fileName,pathName] = uigetfile([pathName '*.*'],'Select Oculus data file','Multiselect','on');

fig = figure;




%A(Loops) = struct('cdata', [],'colormap',[]);

v = VideoWriter('oculus','MPEG-4');
v.FrameRate = 2;
v.Quality = 90;
%v.FileFormat = 'mp4';
open(v)

if iscell(fileName)
  nFiles = length(fileName)
else
  nFiles = 1;
end

for f=1:nFiles
  if nFiles == 1
    dataFileName = fileName;
  else
    dataFileName = fileName{f};
  end
  
  fprintf('\n.Processing file=%s\n',dataFileName);
  
  [filepath,name,ext] = fileparts(dataFileName);
  
  filePathName = [pathName dataFileName];
  
  switch ext
    case '.csv'
      sonarData = importdata(filePathName);
      sonarData = transpose(sonarData);
      % define the field-of-view of the sonar
      imInfo.freqLbl = 'HF';
      imInfo.fov = 60; % 2.1MHz degrees / 1.2MHz 130 deg
      % define the minimum and maximum range of the sonar
      imInfo.rangeY = [0.2 1.0];
      header.gain = 0;
      header.heading = 0;
      header.pitch = 0;
      header.roll = 0;
      header.rangeCount = size(sonarData,1);
      header.beamCount = size(sonarData,2);
      
    case '.dat'
      [sonarData, header] = oculusReadBinaryData( filePathName );
      if isempty(displayRangeY)
        displayRangeY = [0.1 * header.range header.range];
      end
      header
      imInfo.rangeY = [displayRangeY(1) header.range];
      imInfo.displayRangeY = displayRangeY;
      
      imInfo.gain = header.gain;
      if header.freqMode == 1
        imInfo.fov = 130;
        imInfo.freqLbl = 'LF';
      else
        imInfo.fov = 60;
        imInfo.freqLbl = 'HF';
      end
      
  end
  
  if 0
    header.gain = 0;
    header.heading = 0;
    header.pitch = 0;
    header.roll = 0;
    header.rangeCount = 256;
    header.beamCount = 195;
    header.range = 1;
    
    [sonarData] = makeOculusData(1,header.beamCount,header.rangeCount);
    imInfo.displayRangeY = [0.2 header.range];
    imInfo.fov = 130;
    imInfo.freqLbl = 'LF';
    imInfo.rangeY = [displayRangeY(1) header.range];
    
    header.freqMode = 1;
    dataFileName = 'test'
    
  end
    
  if 0
    figure
    %imagesc(20*log10(sonarData))
    imagesc(sonarData)
    ax = gca;
    ax.YDir = 'normal';
    colormap(jet)
    title('Polar data')
    xlabel('beams')
    ylabel('range')
    colorbar
    
    figure
    histogram(sonarData(:),100)
  end
  
  if cfg.swapBytes
    sonarDataMsbyte = floor(sonarData / 256);
    sonarDataLsbyte = sonarData - 256 * sonarDataMsbyte;
    sonarDataSwap = 256 * sonarDataLsbyte + sonarDataMsbyte;
    sonarData = sonarDataSwap;
    if 0
      figure
      subplot(1,3,1)
      imagesc(sonarDataLsbyte)
      ax = gca;
      ax.YDir = 'normal';
      colorbar
      title('LSByte')
      
      subplot(1,3,2)
      imagesc(sonarDataMsbyte)
      ax = gca;
      ax.YDir = 'normal';
      colorbar
      title('MSByte')
      
      subplot(1,3,3)
      imagesc(sonarDataSwap)
      ax = gca;
      ax.YDir = 'normal';
      colorbar
      title('Byte swap')
    end
    
    
  end
  
  writeSonarPng(filePathName,header,uint16(sonarDataSwap));


  % calculate the cross-range distance
  imInfo.rangeX = [-1 1] * sind(imInfo.fov / 2) * max(imInfo.rangeY);

      
  % the number of cross range cells is odd so that it is symmetrical about the centre beam
  % the sonar only has even number of beams?
  imOut = slice2sector(sonarData,imInfo.displayRangeY,imInfo.rangeY,imInfo.fov );
  
  %dynamicRange = cfg.DR 0];
  figure(fig)
  %imagesc(imOut / max(imOut(:)),dynamicRange)
  image2d = imOut / max(imOut(:));
  if cfg.scaling == 2
    image2d = 20*log10(imOut / max(imOut(:)));
    cfg.scalingLbl = 'dB';
  else
    cfg.scalingLbl = 'Lin';
  end
  
  imagesc(imInfo.rangeX,[0 imInfo.displayRangeY(2)],image2d,cfg.DR)
  xlabel('Cross-Range (m)')
  ylabel('Slant Range (m)')
  str1 = sprintf('PingId:%d %s %s Fov:%ddeg Gain:%4.1fdB Rng:%d Beams:%d', ...
    header.pingId,cfg.scalingLbl,imInfo.freqLbl,imInfo.fov,header.gain,header.rangeCount,header.beamCount);
  str2 = sprintf('H:%5.1f P:%5.1f R:%5.1f %s', ...
    header.heading,header.pitch,header.roll,dataFileName);
  
  title({str1 str2})
  ax = gca;
  ax.YDir = 'normal';
  colormap(cfg.map)
  colorbar
  grid
  if cfg.axisSquare
    axis equal
  end
  frame = getframe(gcf);
  %A(f) = frame;
  writeVideo(v,frame)
  
  
end

close(v)
%cfg
%imInfo

