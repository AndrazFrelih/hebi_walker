function [meas_bus] = meas_bus_creator
    meas.zmp = zeros(2,1);
    meas.com = zeros(3,1);
    meas.comp = zeros(3,1);
    meas.compp = zeros(3,1);
    meas.dcm = zeros(3,1);
    meas.dcmp = zeros(3,1);
    meas.Hb_eeL = zeros(4,4);
    meas.Hb_eeR = zeros(4,4);
    meas.Hw_b = zeros(4,4);
    meas.qL = zeros(6,1);
    meas.qR = zeros(6,1);

    meas_info = Simulink.Bus.createObject(meas);
    meas_bus = evalin('base', meas_info.busName);
end

