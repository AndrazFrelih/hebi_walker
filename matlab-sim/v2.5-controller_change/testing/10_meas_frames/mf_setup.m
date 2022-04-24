eul_rot = [0,45,0]*pi/180;
Rref = eul2rotm(eul_rot);
vref_F = [0;0;1];
vref_B = Rref*vref_F;
