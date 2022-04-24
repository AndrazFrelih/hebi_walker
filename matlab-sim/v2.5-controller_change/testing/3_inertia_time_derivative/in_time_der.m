%% (1) trying to prove that Rp*I*R' = R*I*Rp'
syms Ixx Ixy Ixz Iyy Iyz Izz real
I = [
    Ixx, Ixy, Ixz;
    Ixy, Iyy, Iyz;
    Ixz, Iyz, Izz
];

syms A(t) B(t) C(t)
assumeAlso(A(t),'real');
assumeAlso(B(t),'real');
assumeAlso(C(t),'real');

R = rotz(C)*roty(B)*rotx(A);

Ir = R*I*R';

Ir_p = simplify(diff(Ir,t));

Ir_p2 = simplify(2*diff(R,t)*I*R');

%as seen from the following error, (1) does not hold
Err = simplify(Ir_p-Ir_p2);

%% (2) trying to prove that om_sk*Irot = Irot*om_sk'
% Irot is a symmetric matrix Irot - Irot' = 0;
syms omX omY omZ real
om = [omX;omY;omZ];
om_sk = [
    0, omZ, -omY;
    -omZ, 0, omX;
    omY, -omX, 0
];

% (2) does not hold: om_sk*Ir =/= Ir*om_sk' (only holds for vectors)
Err2 = simplify(om_sk*Ir + Ir*om_sk');

%% (3) are both of the terms in the equation om_sk*Ir+Ir*om_sk' important though?
% The answer is NO!
% om_sk'*om == 0, as this is equivalent to a cross product of equal vectors
% result of which is zero: om x om = 0 -> look at term 2
Term1 = simplify(om_sk*Ir*om);
Term2 = simplify(Ir*om_sk'*om);
    