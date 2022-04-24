function [Model] = model_gt2(robot, t, y, qtraj, qptraj, K, D)
    persistent jjj
    if t==0
        jjj = 0;
    end
    jjj=jjj+1;
    
	Cfg = y(1:end/2);
    Tgr = gravityTorque(robot,Cfg);
    TL = Tgr(1:end/2) + K.*(qtraj(1:end/2,jjj)-y(1:end/4)) + D.*(qptraj(1:end/2,jjj)-y(end/2+1:end*3/4));
    TR = Tgr(end/2+1:end) + K.*(qtraj(end/2+1:end,1)-y(end/4+1:end/2)) + D.*(-y(end*3/4+1:end));
    Ttot = [
        TL;
        TR;
    ];

    Model = [
        y(end/2+1:end);
        forwardDynamics(robot,y(1:end/2),y(end/2+1:end),Ttot);
    ];
%     if mod(jjj-1,10)==0
%         show(robot, y(1:end/2));
%         view([90 0]);
%         camproj('orthographic');
%         pause(0.1);
%     end
end

