% floating base variant
if ctrl_cfg_rem.fsm.test_type == -1
    VAR_FB = 0;
else
    VAR_FB = 1;
end

%disturbance
if ctrl_cfg_rem.fsm.test_type == 0
    VAR_DIST = 1;
else
    VAR_DIST = 0;
end
Fdist = [0;20;0;0;0;0];
Tdist = [0.6;0.61];


VAR_TG = 1;
VAR_IK = 1;