function [Model] = model_gt(robot, t, y, qdes, K, D, wcom)
	Cfg = y(1:end/2);
    Tgr = gravityTorque(robot,Cfg);
    TL = Tgr(1:end/2) + K.*(qdes(1:end/2)-y(1:end/4)) + D.*-y(end/2+1:end*3/4);
    TR = Tgr(end/2+1:end);
    Ttot = [
        TL;
        TR;
    ];

    [~,Wext] = get_kine(robot,Cfg,wcom);

    Model = [
        y(end/2+1:end);
        forwardDynamics(robot,y(1:end/2),y(end/2+1:end),Ttot,Wext);
    ];
    
    %show(robot, y(1:end/2));
    %pause(0.1);
end

