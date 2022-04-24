function [J_wrld] = compute_jac_world_single(Hwb,J_base,Hbi,side)
    Rwb = Hwb(1:3,1:3);
    vec = Hbi(1:3,4); %this is the transformation from the point i to the base
    Jbase = [
        eye(3),   -Rwb*skew_mat(vec);
        zeros(3),  Rwb;
    ];

    Jmainleg = [
        Rwb*J_base(1:3,:);
        Rwb*J_base(4:6,:);
    ];

    if side=='l'
        J_wrld = [
            Jbase,Jmainleg,zeros(size(Jmainleg));
        ];
    else
        J_wrld = [
            Jbase,zeros(size(Jmainleg)),Jmainleg;
        ];
    end
end

