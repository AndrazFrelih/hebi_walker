function [ Rz ] = RotZ2( gammaZ )

% TODO: Fill with the proper rotation matrix

Rz=[
    cos(gammaZ),-sin(gammaZ),0,0;
    sin(gammaZ), cos(gammaZ),0,0;
    0,0,1,0;
    0,0,0,1
];

end

