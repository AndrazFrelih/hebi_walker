function [sol_arr_ext] = compute_vels(sol_arr,state,Nstates,Nz,Tsample)
    sol_arr_ext = cell(size(sol_arr));
    
    %% compute velocities (opt. solution are only positions)
    pos = [];
    vels = [];
    for i=1:Nstates
        %compute velocities
        if i==1
            sol_arr_ext{i} = [
                sol_arr{i};
                zeros(Nz,1), (sol_arr{i}(1:Nz,2:end)-sol_arr{i}(1:Nz,1:end-1))/Tsample;
            ];
            pos = sol_arr_ext{i}(1:Nz,:);
            vels = sol_arr_ext{i}(Nz+1:end,:);
        else
            sol_arr_ext{i} = [
                sol_arr{i};
                (sol_arr{i}(1:Nz,1:end)-[sol_arr{i-1}(1:Nz,end),sol_arr{i}(1:Nz,1:end-1)])/Tsample;
            ];
            
            pos = [pos, sol_arr_ext{i}(1:Nz,:)]; %used for a sgolay_test (benchmarking folder)
            vels = [vels,sol_arr_ext{i}(Nz+1:end,:)];
        end
    end
    
    %% filtering
    %velocities have to be filtered (optimization introduces some
    %inaccuracy)
    N_FIR_bits = 2;
    N_FIR = 2^N_FIR_bits-1;
    fg = 5; %empirically investigated with: evaluate_spectrum(sol_arr{1}(Nz+1,:), Tsample) function
    Fenster=ones(N_FIR+1,1);
    B_FIR=fir1(N_FIR,2*fg*Tsample,Fenster);
    B_FIR = B_FIR/sum(B_FIR);
    
    vfil = cell(Nstates,1);
    vfil2 = cell(Nstates,1);
    for i=1:Nz
        %test FIR vs Savinsky Golay filter
        vfil{i} = filter(B_FIR,1,vels(i,:)); %induces a delay
        vfil2{i} = sgolayfilt(vels(i,:),7,21); %performance is better when all the data is known
    end
    
    %which of the velocities should be plotted
    pl_ind = 6;
    plot(vels(pl_ind,:));
    hold on
    plot(vfil{pl_ind});
    plot(vfil2{pl_ind}); %does not induce a delay
    
    %% corrected velocities
    % overwrite the velocities in the solution with filtered values
    % store the computed joint values to the state vector
    ien = 0;
    for i=1:Nstates
        ist = ien+1;
        ien = ien+state{i}.Npts;
        for j=1:Nz
            sol_arr_ext{i}(Nz+j,:) = vfil2{j}(ist:ien);
        end
    end
    
end

