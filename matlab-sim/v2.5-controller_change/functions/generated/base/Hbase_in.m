function [out] = Hbase_in(inp)
x = inp(1);
y = inp(2);
z = inp(3);
rollx = inp(4);
pitchy = inp(5);
yawz = inp(6);

out = zeros(4,4);

out(1,1) = cos(pitchy)*cos(yawz);
out(1,2) = -1.0*cos(pitchy)*sin(yawz);
out(1,3) = sin(pitchy);
out(1,4) = x;

out(2,1) = cos(rollx)*sin(yawz) + cos(yawz)*sin(pitchy)*sin(rollx);
out(2,2) = cos(rollx)*cos(yawz) - 1.0*sin(pitchy)*sin(rollx)*sin(yawz);
out(2,3) = -1.0*cos(pitchy)*sin(rollx);
out(2,4) = y;

out(3,1) = sin(rollx)*sin(yawz) - 1.0*cos(rollx)*cos(yawz)*sin(pitchy);
out(3,2) = cos(yawz)*sin(rollx) + cos(rollx)*sin(pitchy)*sin(yawz);
out(3,3) = cos(pitchy)*cos(rollx);
out(3,4) = z;

out(4,1) = 0;
out(4,2) = 0;
out(4,3) = 0;
out(4,4) = 1.0;

end
