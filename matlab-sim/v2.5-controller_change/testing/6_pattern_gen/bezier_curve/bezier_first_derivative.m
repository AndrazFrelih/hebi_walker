function [vel_trajcetory] = bezier_first_derivative(Pts,tvec)
    n = size(Pts,2)-1;
    c = zeros(n+1,size(tvec,2));
    for i=1:n+1
       c(i,:) = bezier_coeff(n,i-1,tvec); 
    end
    Pts_diff = Pts(:,2:end)-Pts(:,1:end-1);
    vel_trajcetory = [Pts_diff,Pts_diff(:,end)]*c;
end

