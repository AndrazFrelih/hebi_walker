function [jac_bus] = jac_bus_creator
    Jdim = zeros(6,18);
    Jdim_base = zeros(6,6);
    jac.Jw_eeL = Jdim;
    jac.Jw_eeR = Jdim;
    jac.Jw_com = Jdim;
    jac.Jw_b = Jdim;
    jac.Jb_eeL = Jdim_base;
    jac.Jb_eeR = Jdim_base;
    jac_info = Simulink.Bus.createObject(jac);
    jac_bus = evalin('base', jac_info.busName);
end

