% Ping rate not in since the ping rate in the sonar returned messages does
% not change with settings

% FreqMode = 1 for low frequency mode
%          = 2 for high frequency mode

addpath('20211105_15h45m15') %configure 
binaryFilename = "00022.dat";

[data, header, fileVersion] = readbinaryData( binaryFilename );

dataSize = header.rangeCount * header.beamCount;

figure()
imshow(data, []);
xlabel('beams')
ylabel('range')


%% functions
function [data, header, fileVersion] = readbinaryData( filename )
    fid = fopen(filename, 'r', 'l');
    
    skip = 4; % skip 4 bytes
    fileVersion = fread(fid, 1, "uint32", skip);
    
    header.speedOfSoundUsed = fread(fid, 1, "double");
    header.frequency = fread(fid, 1, "double");
    header.temperature = fread(fid, 1, "double");
    header.pressure = fread(fid, 1, "double");
    header.heading = fread(fid, 1, "double");
    header.pitch = fread(fid, 1, "double");
    header.roll = fread(fid, 1, "double");
    header.pingStartTime = fread(fid, 1, "double");
    header.range = fread(fid, 1, "double");
    header.gain = fread(fid, 1, "double");
    header.rangeResolution = fread(fid, 1, "double");
%     header.msgHeaderSize = fread(fid, 1, "int");   % taken out
    header.pingId = fread(fid, 1, "uint32");
    header.dataSize = fread(fid, 1, "uint32");
    header.rangeCount = fread(fid, 1, "uint16");
    header.beamCount = fread(fid, 1, "uint16");
    header.freqMode = fread(fid, 1, "uint8");
    
%     data = fread(fid, Inf, "uint16");
    if header.dataSize == header.rangeCount*header.beamCount*2 % *2 for size of uint16
        data = fread(fid, [header.rangeCount header.beamCount], "uint16");
    else
        error("Sonar image data size and rangCount/beamCount dismatch!")
    end
    
    fclose(fid);
end

%% useful code snippets - using fseek
%  idx1 = ftell(fid);
% fseek(fid, (startingIdx-1 )*2, 'bof');   % -1 because idx starts at 0 for fseek, *2 because "short" is 2 bytes
%  idx2 = ftell(fid);
% data = fread(fid, lengthOfData, 'short');