function [out] = B_XYZ(inp)
rollx = inp(1);
pitchy = inp(2);
yawz = inp(3);

out = zeros(3,3);

out(1,1) = 1;
out(1,2) = 0;
out(1,3) = sin(pitchy);

out(2,1) = 0;
out(2,2) = cos(rollx);
out(2,3) = -cos(pitchy)*sin(rollx);

out(3,1) = 0;
out(3,2) = sin(rollx);
out(3,3) = cos(pitchy)*cos(rollx);

end
