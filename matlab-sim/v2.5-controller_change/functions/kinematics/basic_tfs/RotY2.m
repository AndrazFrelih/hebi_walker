function [ Ry ] = RotY2( betaY )

% TODO: Fill with the proper rotation matrix

Ry=[
    cos(betaY),0,sin(betaY),0;
    0,1,0,0;
    -sin(betaY),0,cos(betaY),0;
    0,0,0,1
];

end

