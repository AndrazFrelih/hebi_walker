function [B,Bp] = eulvel2genvel(eulvar,conv)
    roll = eulvar(1);
    pitch = eulvar(2);
    yaw = eulvar(3);
    roll_p = eulvar(4);
    pitch_p = eulvar(5);
    yaw_p = eulvar(6);

    Rx = RotX(roll);
    Ry = RotY(pitch);
    Rz = RotZ(yaw);
    
    Rwb = sym(eye(3));
    for i=1:3
        ch = conv(i);
        if ch == 'x'
            Rwb = Rwb*Rx;
        elseif ch == 'y'
            Rwb = Rwb*Ry;
        elseif ch == 'z'
            Rwb = Rwb*Rz;
        else
            disp("Error wrong letter used");
        end
    end

    %Rwb = RotX(roll)*RotY(pitch)*RotZ(yaw);
    Rwb_p = diff(Rwb,roll)*roll_p + diff(Rwb,pitch)*pitch_p + diff(Rwb,yaw)*yaw_p;
    W_skew = Rwb_p*Rwb';
    wx_d = -W_skew(2,3);
    wy_d = W_skew(1,3);
    wz_d = -W_skew(1,2);
    
    % transformation matrix between euler velocities and rot. velocities: w = B*ep
    B = simplify([
        diff(wx_d,roll_p),diff(wx_d,pitch_p),diff(wx_d,yaw_p);
        diff(wy_d,roll_p),diff(wy_d,pitch_p),diff(wy_d,yaw_p);
        diff(wz_d,roll_p),diff(wz_d,pitch_p),diff(wz_d,yaw_p)
    ]);

    % time derivative of the matrix B. Used to get rotational accelerations: b = Bp*ep+B*epp
    Bp = simplify(diff(B,roll)*roll_p + diff(B,pitch)*pitch_p + diff(B,yaw)*yaw_p);
end

