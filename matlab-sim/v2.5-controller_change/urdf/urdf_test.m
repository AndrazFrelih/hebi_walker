%execute bipedal_v2_2 first
roby = importrobot("florenceRBDL.urdf"); % here Base is fixed and not considered in COM computation
figure(2);

show(roby,Q_config,'Collisions','off','Visuals','off');
roby.DataFormat = 'column';

Qdef = ones(12,1);
QdefL = Qdef(1:end/2);
QdefR = Qdef(end/2+1:end);

for i=1:7
    disp("Transf L"+i);
    getTransform(roby,Qdef,frame_list.links.lleg(i),frame_list.links.base)
end

for i=1:7
    disp("Transf R"+i);
    getTransform(roby,Qdef,frame_list.links.rleg(i),frame_list.links.base)
end

[HabsL] = H_abs_bL(QdefL);
[HabsR] = H_abs_bR(QdefR);

for i=1:7
    disp("Gen transf. L"+i);
    reshape(HabsL(i,:,:),4,4)
end

for i=1:7
    disp("Gen transf. R"+i);
    reshape(HabsR(i,:,:),4,4)
end



roby2 = importrobot("florenceMatlab.urdf"); %here base is not fixed (so also base com is considered)
roby2.DataFormat = 'column';
disp("COM computation:");
centerOfMass(roby2,Qdef)

x = [0 0 0.6624 0 0 0]';
disp("Derived COM computation:");
compute_com_pos2(x,QdefL,QdefR,mlist,"w")



%for i=1:22
%    roby.Bodies{i}.Name
%end
