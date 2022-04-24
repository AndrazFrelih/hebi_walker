function cont_data = create_cd_struct(zmplocL, zmplocR, FL, FR, TL, TR, cL, cR,stateL,stateR)
    cont_data = struct('L',struct('zmp',zmplocL,'F',FL,'T',TL,'cont',cL,'state',stateL),'R',struct('zmp',zmplocR,'F',FR,'T',TR,'cont',cR,'state',stateR));
end

