function [data, header, fileVersion] = oculusReadBinaryData( filename )
    fid = fopen(filename, 'r', 'l');
    
    skip = 4; % skip 4 bytes
    header.fileVersion = fread(fid, 1, "uint32", skip)
    
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

    if header.fileVersion >= 2
      header.msSinceEpoch = fread(fid, 1, "uint64");
      header.DateTime = datetime(header.msSinceEpoch/1000, 'convertfrom',...
        'posixtime', 'Format', 'MM/dd/yy HH:mm:ss.SSS');

    end
    header.pingId = fread(fid, 1, "uint32");
    header.dataSize = fread(fid, 1, "uint32");
    header.rangeCount = fread(fid, 1, "uint16");
    header.beamCount = fread(fid, 1, "uint16");
    header.freqMode = fread(fid, 1, "uint8");
   
    header.DateTime = datetime(header.msSinceEpoch/1000, 'convertfrom',...
                            'posixtime', 'Format', 'MM/dd/yy HH:mm:ss.SSS');
                          
%     data = fread(fid, Inf, "uint16");
    if header.dataSize == header.rangeCount*header.beamCount*2 % *2 for size of uint16
        data = fread(fid, [header.rangeCount header.beamCount], "uint16");
    else
        error("Sonar image data size and rangCount/beamCount dismatch!")
    end
    
    % byte swap of sonar data
    % CINDY to do
    
    
    fclose(fid);
end
