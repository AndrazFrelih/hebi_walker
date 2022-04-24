function [ikine_inp_bus] = ikine_inp_bus_creator
    ikine_inp.com = zeros(3,1);
    ikine_inp.comp = zeros(3,1);
    ikine_inp.tL = zeros(3,1);
    ikine_inp.tR = zeros(3,1);
    ikine_inp.vL = zeros(3,1);
    ikine_inp.vR = zeros(3,1);
    ikine_inp.qold = zeros(12,1);
    ikine_inp_info = Simulink.Bus.createObject(ikine_inp);
    ikine_inp_bus = evalin('base', ikine_inp_info.busName);
end

