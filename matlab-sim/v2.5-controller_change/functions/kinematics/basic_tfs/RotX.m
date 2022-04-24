function [ Rx ] = RotX( alphaX )

% TODO: Fill with the proper rotation matrix

Rx=[
    1,0,0;
    0,cos(alphaX),-sin(alphaX);
    0,sin(alphaX),cos(alphaX)
];

end

