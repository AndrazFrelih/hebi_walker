function [ Ry ] = RotY( betaY )

% TODO: Fill with the proper rotation matrix

Ry=[
    cos(betaY),0,sin(betaY);
    0,1,0;
    -sin(betaY),0,cos(betaY)
];

end

