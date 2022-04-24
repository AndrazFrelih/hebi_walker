function [sol_arr] = opt_velo(robot_model,state,zini,mlist,options,Ts)
    Nz = size(zini,1)/2;
    Nq = Nz-3;
    Nstates = size(state,1);
    qp_prev = zeros(Nq,1);
    
    %% Linear equality constraints Aleq*z = bleq
    Aleq = [
        -eye(3),     zeros(3,6),  zeros(3,6),  Ts*eye(3),   zeros(3,6),   zeros(3,6);
        zeros(6,3),  -eye(6),     zeros(6),    zeros(6,3),  Ts*eye(6),    zeros(6);
        zeros(6,3),  zeros(6),    -eye(6),     zeros(6,3),  zeros(6),     Ts*eye(6);
    ];
    %rhs vector bleq changes in each iteration
    bleq = -zini(1:end/2);
    
    
    %% setup the rest of the optimization
    % allocate memory for the solution
    sol_arr = cell(size(state,1),1);
    for i=1:Nstates
        %the dimensionality is two times bigger as velocities are included
        sol_arr{i} = zeros(2*Nz,state{i}.Npts);
    end

    %iterate through all the states
    stPh = 1;
    for i = stPh:Nstates
        curr_st = state{i};
        curr_ph = curr_st.phase;
        %shift weight only matters in SS phase
        if curr_st.shift_weight.val<0
            curr_sw = 'r';
        elseif curr_st.shift_weight.val>0
            curr_sw = 'l';
        else
            curr_sw = 'm';
        end

        Ni_pts = curr_st.Npts;
        for j = 1:Ni_pts
            tic
            %% nonlinear inequality constraint tasks
            if curr_ph=='d'
                %both feet are in full contact with the ground
                Fk_task_constr = [
                    %left task
                    curr_st.footstep.Ltraj.x(1,j);
                    curr_st.footstep.Ltraj.x(2,j);
                    zeros(4,1);
                    %right task
                    curr_st.footstep.Rtraj.x(1,j);
                    curr_st.footstep.Rtraj.x(2,j);
                    zeros(4,1);
                ];
            else
                if curr_sw=='l'
                    %left foot is the support foot
                    Fk_task_constr = [
                        %left task
                        curr_st.footstep.Ltraj.x(1:2,j);
                        zeros(4,1);
                        %right task is not active (set to zero on both sides)
                        zeros(6,1);
                    ];                
                elseif curr_sw=='r'
                    %right foot is the support foot
                    Fk_task_constr = [
                        %left task is not active (set to zero on both sides)
                        zeros(6,1);
                        %right task
                        curr_st.footstep.Rtraj.x(1:2,j);
                        zeros(4,1);
                    ];  
                end
            end

            nlcstr_fcn = @(z)nlcstr_mot_pl(z, curr_ph, curr_sw, Fk_task_constr);

            %% cost function tasks
            if curr_ph=='s'
                % only for single support we have tasks in a cost function
                if curr_sw=='l'
                    % right leg is swinging
                    % forward kinematics (position profile)
                    Fk_task_cf = [
                        curr_st.footstep.Rtraj.x(1:2,j);            %x,y position
                        ones(4,1)*curr_st.footstep.Rtraj.x(3,j);    %z positions of 4 contact points
                    ];
                    % forward differential kinematics (velocity profile)
                    FDk_task_cf = [
                        curr_st.footstep.Rtraj.xp(1:3,j);           %tra. velocity
                        zeros(3,1);                                 %rot. velocity        
                    ];
                elseif curr_sw=='r'
                    % left leg is swinging
                    % forward kinematics (position profile)
                    Fk_task_cf = [
                        curr_st.footstep.Ltraj.x(1:2,j);            %x,y position
                        ones(4,1)*curr_st.footstep.Ltraj.x(3,j);    %z positions of 4 contact points
                    ];
                    % forward differential kinematics (velocity profile)
                    FDk_task_cf = [
                        curr_st.footstep.Ltraj.xp(1:3,j);           %tra. velocity
                        zeros(3,1);                                 %rot. velocity 
                    ];
                end
            else    
                %not considered (input any value)
                Fk_task_cf = 0;
                FDk_task_cf = 0;
            end

            curr_C_des = state{i}.com.x(:,j);
            curr_Cp_des = state{i}.com.xp(:,j);

            %generate the cost function
            cost_fcn = @(z)costfun_mot_pl(z, curr_ph, curr_sw, curr_C_des, curr_Cp_des, Fk_task_cf, FDk_task_cf, qp_prev, mlist);

            %% rhs of the linear eq. equation Aleq*z = bleq
            if i~=stPh && j~=1
                bleq = -zsol(1:end/2);
            end
            
            %% solve the problem
            
            [zsol,~] = fmincon(cost_fcn,zini,[],[],Aleq,bleq,[],[],nlcstr_fcn,options);
            %[zsol,~] = fmincon(cost_fcn,zini,[],[],[],[],[],[],nlcstr_fcn,options);
            
            %store solution
            sol_arr{i}(:,j) = zsol;
            
            %extract previous velocities
            qp_prev = zsol(end-Nq+1:end);
            zini = zsol;
            toc
        end
    end
end

