function [Ftarg,zmp_glo,zmp_loc,state] = force_remapping(Forig,AinvX,AinvY,Apseud,Porig,Ptarg, Rfoot2world, tfoot2world,thr,frame)
    Ftarg = [0;0;0;0]; %measurement forces that get fed to the controller
    state = 0; %detected contact state
    
    f = find(Forig>thr); %always returns indices in ascending order
    if ~isempty(f)
        Nf = max(size(f));
    else
        Nf = 0;
    end
    
    zmp_loc = zeros(3,1);
    
    if Nf==1
        %vertex contact
        Ftarg(f) = Forig(f);
        if frame=='o'
            zmp_loc = [Porig(f,:)';0];
        elseif frame=='t'
            zmp_loc = [Ptarg(f,:)';0];
        end
        state = 1;
    elseif Nf==2   
        %edge contact
        if (f(1)==1 && f(2)==2) || (f(1)==3 && f(2)==4)
            %edge aligned with Y-axis
            j=2;
            
            %take care that first element of the vector is the LEFT one
            if(f(1)==1)
                %F1 and then F2
                F = [Forig(f(1));Forig(f(2))];
            else
                %F4 and then F3
                F = [Forig(f(2));Forig(f(1))];
            end
            
            %compute the linear coefficients
            K = AinvY*F;
            
            %update the measurements
            for i=1:2
                Ftarg(f(i)) = dot(K,[Ptarg(f(i),j),1]);
            end
            
            %compute the center of pressure
            if frame=='o'
                px = Porig(f(1),1); %edge of the foot in x-dir
                py_num = Porig(f(1),2)*Forig(f(1))+Porig(f(2),2)*Forig(f(2));
                py_den = Forig(f(1))+Forig(f(2));
                py = py_num/py_den;
                zmp_loc = [px;py;0];
            elseif frame=='t'
                px = Ptarg(f(1),1); %edge of the foot in x-dir
                py_num = Ptarg(f(1),2)*Ftarg(f(1))+Ptarg(f(2),2)*Ftarg(f(2));
                py_den = Ftarg(f(1))+Ftarg(f(2));
                py = py_num/py_den;
                zmp_loc = [px;py;0];
            end
        else
            %edge aligned with X-axis
            j=1;
            
            %take care that first element of the vector is the TOP one
            %(always the case as possible combinations here are 1&4 and 2&3)
            F = [Forig(f(1));Forig(f(2))];
            
            %x edge common
            K = AinvX*F;
            
            %update the measurements
            for i=1:2
                Ftarg(f(i)) = dot(K,[Ptarg(f(i),j),1]);
            end
            
            %compute the center of pressure
            if frame=='o'
                py = Porig(f(1),2);
                px_num = Porig(f(1),1)*Forig(f(1))+Porig(f(2),1)*Forig(f(2));
                px_den = Forig(f(1))+Forig(f(2));
                px = px_num/px_den;
                zmp_loc = [px;py;0];
            elseif frame=='t'
                py = Ptarg(f(1),2);
                px_num = Ptarg(f(1),1)*Ftarg(f(1))+Ptarg(f(2),1)*Ftarg(f(2));
                px_den = Ftarg(f(1))+Ftarg(f(2));
                px = px_num/px_den;
                zmp_loc = [px;py;0];
            end
        end
        state = 2;
        
    elseif Nf==3 || Nf==4
        %full foot contact
        K = Apseud*reshape(Forig,4,1);
        for i=1:4
            Ftarg(i) = dot(K,[Ptarg(i,:),1]);
        end
        
        %compute the center of pressure (4 points)
        if frame=='o'
            % compute the zmp in the origin frame when target is Pmeas
            % (simulation)
            px_num = dot(Forig,Porig(:,1));
            py_num = dot(Forig,Porig(:,2));
            p_den = sum(Forig);
            px = px_num/p_den;
            py = py_num/p_den;
            zmp_loc = [px;py;0];
        elseif frame=='t'
            % compute the zmp in the target frame when target is Pedge
            % (controller)
            px_num = dot(Ftarg,Ptarg(:,1));
            py_num = dot(Ftarg,Ptarg(:,2));
            p_den = sum(Ftarg);
            px = px_num/p_den;
            py = py_num/p_den;
            zmp_loc = [px;py;0];
        end
        
        state = 4;
    end
    
    
    zmp_tmp = Rfoot2world*zmp_loc+tfoot2world;
    zmp_glo = [zmp_tmp(1);zmp_tmp(2)];  %the real zmp is only provided for the reference
    zmp_loc = zmp_loc(1:2);
end

