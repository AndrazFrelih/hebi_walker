function [tg_out_bus] = tg_out_bus_creator
    tg_out.zmp = zeros(2,1);
    tg_out.com = zeros(3,1);
    tg_out.comp = zeros(3,1);
    tg_out.compp = zeros(3,1);
    tg_out.dcm = zeros(3,1);
    tg_out.dcmp = zeros(3,1);

    tg_out_info = Simulink.Bus.createObject(tg_out);
    tg_out_bus = evalin('base', tg_out_info.busName);   
end

