function [acc_trajcetory] = bezier_second_derivative(Pts,tvec)
    n = size(Pts,2)-2;
    c = zeros(n+1,size(tvec,2));
    for i=1:n+1
       c(i,:) = bezier_coeff(n,i-1,tvec); 
    end
    Pts_diff = Pts(:,3:end)-2*Pts(:,2:end-1)+Pts(:,1:end-2);
    acc_trajcetory = [Pts_diff,Pts_diff(:,end)]*c;
end

