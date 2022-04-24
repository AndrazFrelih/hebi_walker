function [sol_arr] = opt_stat(robot_model,state,zini,mlist,options,Ts)
    Nz = size(zini,1);
    Nstates = size(state,1);
    
    %% setup the rest of the optimization
    % allocate memory for the solution
    sol_arr = cell(size(state,1),1);
    for i=1:Nstates
        %the dimensionality is two times bigger as velocities are included
        sol_arr{i} = zeros(Nz,state{i}.Npts);
    end
    
    for i = 1:Nstates
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
            %tic
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

            %change this part to be static
            nlcstr_fcn = @(z)nlcstr_mot_pl_static(z, curr_ph, curr_sw, Fk_task_constr);

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
                elseif curr_sw=='r'
                    % left leg is swinging
                    % forward kinematics (position profile)
                    Fk_task_cf = [
                        curr_st.footstep.Ltraj.x(1:2,j);            %x,y position
                        ones(4,1)*curr_st.footstep.Ltraj.x(3,j);    %z positions of 4 contact points
                    ];
                end
            else    
                %not considered (input any value)
                Fk_task_cf = 0;
            end

            curr_C_des = state{i}.com.x(:,j);

            %CHANGE THIS FUNCTION TO BE STATIC
            %generate the cost function
            cost_fcn = @(z)costfun_mot_pl_static(z, curr_ph, curr_sw, curr_C_des, Fk_task_cf, mlist);

            
            %% solve the problem
            [zsol,~] = fmincon(cost_fcn,zini,[],[],[],[],[],[],nlcstr_fcn,options);
            
            %store solution
            sol_arr{i}(:,j) = zsol;
            
            %for the hot start of the optimization
            zini = zsol;
            %toc
        end
    end
end

