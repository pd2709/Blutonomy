function writeSonarPng(fileName,header,sonarData)

  [filepath,name,ext] = fileparts(fileName);

fileName = [filepath '\' name '.png']

imwrite(sonarData,fileName)