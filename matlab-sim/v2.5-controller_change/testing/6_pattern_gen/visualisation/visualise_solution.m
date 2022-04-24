function visualise_solution(robot_model,sol,state,static,Tsample,ang)
    fsample = 1/Tsample;
    Ndrop = fsample/10;
    robot_model.DataFormat = 'col';
    figure(5);
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    %set(4,'DefaultFigureWindowStyle','docked');
    pause(1);
    count = 1;
    for i = 1:size(state,1)
        for j = 1:size(sol{i},2)
            if ~static
                Qalg = sol{i}(4:end/2,j);
            else
                Qalg = sol{i}(4:end,j);
            end
            if mod(count-1,Ndrop)==0
                visualise_robot(robot_model,Qalg,ang);
                pause(0.1);
            end
            count = count +1;
        end
    end

end

