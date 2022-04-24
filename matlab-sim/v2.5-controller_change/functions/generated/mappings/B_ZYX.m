function [out] = B_ZYX(inp)
rollx = inp(1);
pitchy = inp(2);
yawz = inp(3);

out = zeros(3,3);

out(1,1) = cos(pitchy)*cos(yawz);
out(1,2) = -sin(yawz);
out(1,3) = 0;

out(2,1) = cos(pitchy)*sin(yawz);
out(2,2) = cos(yawz);
out(2,3) = 0;

out(3,1) = -sin(pitchy);
out(3,2) = 0;
out(3,3) = 1;

end
