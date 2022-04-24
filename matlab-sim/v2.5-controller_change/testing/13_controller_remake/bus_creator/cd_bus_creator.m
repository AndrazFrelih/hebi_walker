function [cd_bus] = cd_bus_creator
    cd.L.zmp = zeros(3,1);
    cd.L.F = zeros(3,1);
    cd.L.T = zeros(3,1);
    cd.L.cont = 0;
    cd.L.state = 0;
    cd.R.zmp = zeros(3,1);
    cd.R.F = zeros(3,1);
    cd.R.T = zeros(3,1);
    cd.R.cont = 0;
    cd.R.state = 0;
    
    cd_info = Simulink.Bus.createObject(cd);
    cd_bus = evalin('base', cd_info.busName);
end

