function [Jac,Wext,Wlleg] = get_kine(roby,Cfg,wcom)
    Tf = roby.getTransform(Cfg,'EE_L','FloBase');
    AdTf_T = Ad_g2(Tf);
    Wlleg = AdTf_T*wcom;
    %if the wrench is defined in EE frame, Cfg needs to be specified
    Wext = roby.externalForce('EE_L',[wcom(4:6);wcom(1:3)]);
    JacI = roby.geometricJacobian(Cfg,'EE_L');
    Jac = [
        JacI(4:6,1:end/2);
        JacI(1:3,1:end/2);
    ];
end

