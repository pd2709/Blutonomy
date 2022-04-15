function imOut = slice2sector(imRangeBeams,displayRange,rangeLimits,fov)
% Inputs
%   imRect [numSlantRangePoints x numAzBeams]
%   rangeMin (m)
%   rangeLimits [2 x 1] min and max range in Y axis of the input image imRect
%   outXPixels number of pixels in X direction
% Code was at F:\syncMyWork\matlab\data\petrelmk2\src

dbg = 0;


if dbg
  figure
  imagesc(20*log10(imRangeBeams))
  ax = gca;
  ax.YDir = 'normal';
  colormap(jet)
  title('Polar data')
  xlabel('beams')
  ylabel('range')
end

nBeams = size(imRangeBeams,2);
switch 5
  case 1
    outXPixels = [];
    yMax = round(size(imRangeBeams,1) * displayRange(2) / (rangeLimits(2) - rangeLimits(1)));
    
    yMin = 1 + round(yMax * (displayRange(1) - rangeLimits(1)) / (rangeLimits(2) - rangeLimits(1)));
    
    xMax = size(imRangeBeams,2);
    
    xCentre = (xMax + 1) / 2;
    
    imOut = NaN(yMax-yMin+1,outXPixels);
    xStepSize = xMax / outXPixels;
    imOutXcentre = (outXPixels + 1) / 2;
    
    
    yImOut = yMin;
    for y = yMin:yMax
      y;
      
      r = rangeLimits(1) + (y / yMax) * (rangeLimits(2) - rangeLimits(1));
      
      xWidth = (outXPixels - 1) * r / rangeLimits(2);
      imOutXsliceIndex = round(xWidth / 2);
      imOutXsliceIndex = -imOutXsliceIndex:imOutXsliceIndex;
      imSampled = interp1(-(xCentre - 1):(xCentre - 1),imRangeBeams(y,:),(xCentre-1) * imOutXsliceIndex / max(imOutXsliceIndex), 'pchip');
      
      
      %sliceXwidthHalf = round( (xWidth - 1) / 2);
      
      
      
      %sliceXsamples = (-sliceXwidthHalf : sliceXwidthHalf);
      %sliceXsamples = (-sliceXwidthHalf : xStepSize : sliceXwidthHalf);
      %imOutNumHalfSamples = round(sliceXwidthHalf / xStepSize);
      %sliceXsamples = (-imOutNumHalfSamples : imOutNumHalfSamples) * xStepSize;
      
      %imSampled = interp1(-(xCentre - 1):(xCentre - 1),imRect(y,:),sliceXsamples, 'pchip');
      
      %   imOutXsliceWidth = length(imSampled);
      %   imOutXsliceIndex = -round((imOutXsliceWidth-1)/2) : round((imOutXsliceWidth-1) /2);
      %
      %   if min(xCentre + sliceXsamples) < 1
      %     sliceXsamples;
      %   end
      %   if max(xCentre + sliceXsamples) > 61
      %     sliceXsamples;
      %   end
      
      %imOut(y,xCentre + sliceXsamples) = imSampled;
      imOut(yImOut,imOutXcentre + imOutXsliceIndex) = imSampled;
      yImOut = yImOut + 1;
    end
    
  case 2
    yMax = size(imRect,1)
    yMin = 0
    xRange = round(yMax * cosd(fov/2))
    xRange = [-xRange xRange]
    xMax = size(imRect,2)
    xPos = (1:xMax) - xMax/2 - 0.5;
    
    xMat = repmat(xPos,yMax,1);
    yMat = repmat(transpose((0:yMax-1)),1,xMax);
    
    xyBeam = atan2d(xMat,yMat);
    
  case 3
    rMax = size(imRangeBeams,1)
    nBeams = size(imRangeBeams,2)
    rMin = round(0.2*195)
    xWidth = 2 * ceil(rMax * sind(fov/2))
    xMid = xWidth/2 + 1/2;
    %xRange = [-xRange xRange]
    
    %xMax = size(imRangeBeams,2)
    beams = fov/2 * ((1:nBeams) - nBeams/2 - 0.5) / (nBeams/2);
    
    yImOut = rMin;
    imOut = NaN(rMax-rMin+1,xWidth);

    for r = rMin:rMax
      r
      x = sind(beams) * r
      x = round(x + xMid);
      
      y = round(cosd(beams) * r)
      
      for b=1:nBeams
        imOut(y(b),x()) = imRangeBeams(r,b);
      end
    end
    
  case 4
    rMax = size(imRangeBeams,1)
    nBeams = size(imRangeBeams,2)
    beams = 1:nBeams;
    polar.theta = ((0:nBeams-1) - nBeams/2 + 1/2) * fov/nBeams

    rMin = round(rangeLimits(1)*rMax);
    
    xWidth = 2 * ceil(rMax * sind(fov/2));
    xPos = (1:xWidth) - (xWidth+1)/2;
    yMin = rMin * cosd(fov/2);
    
    imOut = NaN(rMax,xWidth);
    dbg = 0;
    %figure
    for y = round(yMin):rMax
      y;
      r = round(sqrt(xPos.^2 + y^2));
      cartesian.theta = atan2d(xPos,y);

      %figure;plot(xPos,r);xlabel('xpos');ylabel('Range');title(sprintf('y=%d',y))
      %figure;plot(xPos,cartesian.theta);xlabel('xpos');ylabel('theta');title(sprintf('y=%d',y))
      
      cartesian.thetaBeam = cartesian.theta/(fov / (nBeams-1));
      cartesian.thetaValidIndex = find(abs(cartesian.theta) < fov/2);
      cartesian.rValidIndex = find((r > rMin) & (r <= rMax));
      cartesian.rThetaIndexValid = intersect(cartesian.thetaValidIndex,cartesian.rValidIndex);
      
      %figure;plot(cartesian.thetaBeam,r,'x');xlabel('thetaBeam');ylabel('r');title(sprintf('y=%d',y))
      polar.rangeAtBeam = interp1(cartesian.thetaBeam,r,(beams-1) - nBeams/2 + 1/2);
      %figure;plot(polar.rangeAtBeam,'x');xlabel('beams');ylabel('Range');title(sprintf('y=%d',y))
      %figure;plot(polar.theta,polar.rangeAtBeam,'x');xlabel('beam theta (deg)');ylabel('Range');title(sprintf('y=%d',y))
       
      polar.indexValid = find((polar.rangeAtBeam <= rMax) & (polar.rangeAtBeam >= 1));

      % index start at 0
      polar.index = round(polar.rangeAtBeam(polar.indexValid)) - 1 + (beams(polar.indexValid)-1)*rMax;
      polar.dBeams = zeros(size(beams));
      polar.dBeams(1,polar.indexValid) = imRangeBeams(polar.index + 1);
      %figure;plot(polar.dBeams);xlabel('beams');ylabel('data');title(sprintf('y=%d',y))
      polar.indexValidSpan = polar.indexValid(end)-polar.indexValid(1);
      
      if ~isempty(cartesian.rThetaIndexValid)
        cartesian.rThetaIndexValidSpan = cartesian.rThetaIndexValid(end) - cartesian.rThetaIndexValid(1);
        
        %figure;plot([0:length(polar.indexValid)-1],polar.dBeams(polar.indexValid));xlabel('beam(0..)');ylabel('data');title(sprintf('y=%d',y))
        
        d = interp1(0:length(polar.indexValid)-1, ...
          polar.dBeams(polar.indexValid), ...
          polar.indexValidSpan*(cartesian.rThetaIndexValid - cartesian.rThetaIndexValid(1)) / cartesian.rThetaIndexValidSpan);
        %figure;plot(d);xlabel('cartesian samples');title(sprintf('y=%d',y))
        
        imOut(y,cartesian.rThetaIndexValid) = d;
      end
      
      
    end
      
  case 5
    rMax = size(imRangeBeams,1);
    nBeams = size(imRangeBeams,2);
    beams = 1:nBeams;
    polar.theta = ((0:nBeams-1) - nBeams/2 + 1/2) * fov/nBeams;

    rMin = rMax * rangeLimits(1) / rangeLimits(2);
    
    xWidth = 2 * ceil(rMax * sind(fov/2));
    xPos = (1:xWidth) - (xWidth+1)/2;
    yMin = rMin * cosd(fov/2);
    
    imOut = zeros(rMax,xWidth);

    %figure
    cartesian.r = NaN(rMax,xWidth);
    cartesian.theta = NaN(rMax,xWidth);

    for y = round(yMin):rMax
      y;
      cartesian.r(y,:) = round(sqrt(xPos.^2 + y^2));
      cartesian.theta(y,:) = atan2d(xPos,y);
    end    
    cartesian.thetaValidIndex = find(abs(cartesian.theta) < fov/2);
    cartesian.rValidIndex = find((cartesian.r > rMin) & (cartesian.r <= rMax));
    cartesian.rThetaIndexValid = intersect(cartesian.thetaValidIndex,cartesian.rValidIndex);

    polar.r = cartesian.r(cartesian.rThetaIndexValid);
    polar.theta = cartesian.theta(cartesian.rThetaIndexValid);
    polar.thetaBeam = nBeams/2 + round(0.5 + polar.theta / (fov/(nBeams-1)));
    polar.index = sub2ind(size(imOut),polar.r,polar.thetaBeam);
    
    imOut(cartesian.rThetaIndexValid) = imRangeBeams(polar.index);
    
    
      
end

if dbg
  figure

  imagesc(20*log10(imOut))
  ax = gca;
  ax.YDir = 'normal';
  colormap(jet)
  axis equal
end