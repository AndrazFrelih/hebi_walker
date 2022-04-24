roby = importrobot("florenceMatlab.urdf");
roby.DataFormat = 'column';

for i=1:Nop_st
    if mod(i-1,100)==0
        show(roby,qfull(7:end,i),"Visuals","on");
        view([90 0]);
        camproj('orthographic');
        axis equal;
        grid on;
        pause(0.01);
    end
end