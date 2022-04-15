function data = makeOculusData(type,nRange,nBeams)

switch type
  case 1
    data = abs(0.01 * randn(nRange,nBeams));
    
    data(:,[ 21  41  61  81 101]) = 1;
    data(:,[236 216 196 176 156]) = 1;
    
    data(51 + (0:2)*50,:) = 0.2;
end
