function [out] = Bp_ZYX(inp)
rollx = inp(1);
pitchy = inp(2);
yawz = inp(3);
rollx_p = inp(4);
pitchy_p = inp(5);
yawz_p = inp(6);

out = zeros(3,3);

out(1,1) = - pitchy_p*cos(yawz)*sin(pitchy) - yawz_p*cos(pitchy)*sin(yawz);
out(1,2) = -yawz_p*cos(yawz);
out(1,3) = 0;

out(2,1) = yawz_p*cos(pitchy)*cos(yawz) - pitchy_p*sin(pitchy)*sin(yawz);
out(2,2) = -yawz_p*sin(yawz);
out(2,3) = 0;

out(3,1) = -pitchy_p*cos(pitchy);
out(3,2) = 0;
out(3,3) = 0;

end
