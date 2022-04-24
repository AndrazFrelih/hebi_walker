roby = importrobot("florenceMatlab.urdf");
roby.DataFormat = 'column';
roby.Gravity = [0,0,-9.81];

G_init = gravityTorque(roby,Q_init);


qstart = Q_init;
%qstart = zeros(12,1);
qdes = qstart;
x1 = zeros(size(qstart));
x2 = zeros(size(qstart));


yinit = [
    qstart;
    zeros(size(qstart));
];

tspan = 0:0.01:2;

Mod = @(t,y) Model(roby,t,y,qdes);

[ysol_rt] = ode1(Mod,tspan,yinit);
ysol_rt = ysol_rt';

fps = 10;
nfr = tspan(end)*fps;
Npts = size(ysol_rt,2);
fact = round(Npts/nfr);
for i = 1: size(tspan,2)
    if mod(i-1,fact) == 0
        show(roby, ysol_rt(1:end/2,i));
        pause(0.1);
    end
end
