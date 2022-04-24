function [out] = Bp_XYZ(inp)
rollx = inp(1);
pitchy = inp(2);
yawz = inp(3);
rollx_p = inp(4);
pitchy_p = inp(5);
yawz_p = inp(6);

out = zeros(3,3);

out(1,1) = 0;
out(1,2) = 0;
out(1,3) = pitchy_p*cos(pitchy);

out(2,1) = 0;
out(2,2) = -rollx_p*sin(rollx);
out(2,3) = pitchy_p*sin(pitchy)*sin(rollx) - rollx_p*cos(pitchy)*cos(rollx);

out(3,1) = 0;
out(3,2) = rollx_p*cos(rollx);
out(3,3) = - pitchy_p*cos(rollx)*sin(pitchy) - rollx_p*cos(pitchy)*sin(rollx);

end
