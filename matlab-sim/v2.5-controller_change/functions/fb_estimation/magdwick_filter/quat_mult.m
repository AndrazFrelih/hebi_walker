function [qC] = quat_mult(qA,qB)
    qC = zeros(4,1);
    qC(1) = qA(1)*qB(1) - qA(2)*qB(2) - qA(3)*qB(3) - qA(4)*qB(4);
    qC(2) = qA(1)*qB(2) + qA(2)*qB(1) + qA(3)*qB(4) - qA(4)*qB(3);
    qC(3) = qA(1)*qB(3) - qA(2)*qB(4) + qA(3)*qB(1) + qA(4)*qB(2);
    qC(4) = qA(1)*qB(4) + qA(2)*qB(3) - qA(3)*qB(2) + qA(4)*qB(1);
end

