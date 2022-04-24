function [trajectory] = bezier_curve(Pts,tvec)
    n = size(Pts,2);
    c = zeros(n+1,size(tvec,2));
    for i=1:n+1
       c(i,:) = bezier_coeff(n,i-1,tvec); 
    end
    trajectory = [Pts,Pts(:,end)]*c;
end

