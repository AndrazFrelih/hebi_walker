function [ph_data_bus] = ph_data_bus_creator
    ph_data.w = zeros(2,1);
    ph_data.phase = 0;
    ph_data_info = Simulink.Bus.createObject(ph_data);
    ph_data_bus = evalin('base', ph_data_info.busName);
end

