function [cdf_data] = zmp_cdf(foot,total_mass,visualise)
    %the condition for a reasonably small measurement errors:
    n_var = 5; %implicitly sets the trust region of the zmp
    zmp_var = min(foot.dims(1:2))/n_var;

    npts = 30;
    xmax = +foot.dims(1)/2;
    xmin = -foot.dims(1)/2;
    dx = foot.dims(1)/(npts-1);
    xvec = [xmin:dx:xmax]';

    ymax = +foot.dims(2)/2;
    ymin = -foot.dims(2)/2;
    dy = foot.dims(2)/(npts-1);
    yvec = [ymin:dy:ymax]';

    Fpdf = @(x,mean,var) 1/sqrt(2*pi*var^2)*exp(-(x-mean)^2/(2*var^2));

    Fpdf_zx = zeros(npts,1);
    Fpdf_zy = zeros(npts,1);

    for i=1:npts
        for j=1:npts
            Fpdf_zx(i) = Fpdf_zx(i) + Fpdf(xvec(i),xvec(j),zmp_var)*dx;
            Fpdf_zy(i) = Fpdf_zy(i) + Fpdf(yvec(i),yvec(j),zmp_var)*dy;
        end
    end

    [Fpdf_zx_mat,Fpdf_zy_mat] = meshgrid(Fpdf_zx, Fpdf_zy);
    Pzmp = Fpdf_zx_mat.*Fpdf_zy_mat;
    Pzmp = Pzmp-min(Pzmp,[],'all');
    Pzmp = Pzmp/max(Pzmp,[],'all');
    [xmat,ymat] = meshgrid(xvec,yvec); % x left to right, y up down
    
    if visualise
        figure(1);
        surf(xvec,yvec,Pzmp);
    end
        
    %% Contact force weights
    npts_F = npts * 5;
    g = 9.81;
    Fmin = 5;
    Fcut = total_mass * g * 1;
    Fmax = total_mass * g * 3;
    Fpdf_cont = zeros(npts_F,1);
    dF = (Fmax - Fmin)/(npts_F-1);
    Fvec = Fmin:dF:Fmax;

    Fcont_var = (Fmax - Fmin)/(2.25*Fmin); %max temperature error is 1.25Fmin and nonlinearity/hysteresis error is 1Fmin

    for i=1:npts_F
        for j=1:npts_F
            Fpdf_cont(i) = Fpdf_cont(i) + Fpdf(Fvec(i),Fvec(j),Fcont_var)*dF;
        end
    end
    Pcont = Fpdf_cont;
    Pcont = Fpdf_cont-min(Pcont,[],'all');
    Pcont = Pcont/max(Pcont,[],'all');

    Fvec_new = Fvec(Fvec<Fcut);
    Pcont_new = Pcont(Fvec<Fcut);
    
    if visualise
        figure(2);
        plot(Fvec_new,Pcont_new);
    end
    
    %generate output
    cdf_data.cont.f = Fvec_new;
    cdf_data.cont.p = Pcont_new;
    cdf_data.zmp.x = xmat;
    cdf_data.zmp.y = ymat;
    cdf_data.zmp.p = Pzmp;
end