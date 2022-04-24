function vis_gt(roby,tspan,ysol_rt)
    fps = 30;
    nfr = tspan(end)*fps;
    Npts = size(ysol_rt,2);
    fact = round(Npts/nfr);
    for i = 1: size(tspan,2)
        if mod(i-1,fact) == 0
            show(roby, ysol_rt(1:end/2,i));
            view([90 0]);
            camproj('orthographic');
            pause(0.1);
        end
    end
end

