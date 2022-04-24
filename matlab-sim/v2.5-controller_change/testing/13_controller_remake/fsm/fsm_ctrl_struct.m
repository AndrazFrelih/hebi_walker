function [FSM] = fsm_ctrl_struct
	FSM = struct('pg_act',0,'tg',struct('act',0,'use_fb',0),'est_act',1,'bal',struct('act',0,'type',1),'ikine',0,'gs',struct('reduce_gains',0));
end

