function [traj] = generate_foot_traj(tpts,tvec,Xst,Xen,Zstep)
    if Zstep == 0
        x = generate_lin_traj(tpts,tvec,Xst,Xen);
        xp = zeros(size(x));
        xpp = zeros(size(x));
    else
        fcn = @(p,vst,ven)vst+p.*(ven-vst);
        
        Liftoff = [0;0;Zstep];
        Xst_int = fcn(0.4,Xst,Xen) + Liftoff;
        Xen_int = fcn(0.6,Xst,Xen) + Liftoff;
        Xen_int2 = fcn(0.8,Xst,Xen) + Liftoff/3;
        Xen_int3 = fcn(0.9,Xst,Xen) + Liftoff/10;
        Xen_int4 = fcn(0.95,Xst,Xen) + Liftoff/20;
        %so many starting and ending points force starting and ending accelerations to be zero

        
        Ctrl_pts = [
            Xst,Xst,Xst_int,Xen_int,Xen_int2,Xen_int3,Xen_int4,Xen,Xen
            %Xst,Xst_int,Xen_int,Xen
        ];
        
        tvec_n = tvec/tvec(end);
        
        [x,xp,xpp] = bezier_full_trajectory(Ctrl_pts,tvec_n);
    end
    traj = struct("x",x,"xp",xp,"xpp",xpp,"t",tvec);
end

