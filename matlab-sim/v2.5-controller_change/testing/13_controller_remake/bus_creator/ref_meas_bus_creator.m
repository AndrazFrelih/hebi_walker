function [ref_meas_bus] = ref_meas_bus_creator
    ref_meas.zmp = zeros(2,1);
    ref_meas.com = zeros(3,1);
    ref_meas.comp = zeros(3,1);
    ref_meas.dcm = zeros(3,1);
    ref_meas.tfb = zeros(3,1);
    ref_meas.Rfb = zeros(3,3);

    ref_meas_info = Simulink.Bus.createObject(ref_meas);
    ref_meas_bus = evalin('base', ref_meas_info.busName);
end

