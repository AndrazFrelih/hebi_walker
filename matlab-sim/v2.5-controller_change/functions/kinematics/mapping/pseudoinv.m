function [Jpseud] = pseudoinv(A,rho,side)
    if strcmp(side,'left')
        %overactuated robots (tasks have less DOF than robots)
        Jpseud = (A'*A+rho^2*eye(max(size(A))))\A';  
    elseif strcmp(side,'right')
        %underactuated robots (tasks have more DOF than robots)
        Jpseud = A'/(A*A'+rho^2*eye(min(size(A))));
    end
end

