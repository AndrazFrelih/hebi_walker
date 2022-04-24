function create_symbolic_function2(name,path,expr,var_list,const_list)
    str = path+name+".m";
    s = size(expr);
    ss = max(size(s));
    fid = fopen(str,"w");
    if size(const_list)
        fprintf(fid,"function [out] = %s(inp,consts)\n",char(name));
    else
        fprintf(fid,"function [out] = %s(inp)\n",char(name));
    end
    for i=1:max(size(var_list))
        fprintf(fid,"%s = inp(%d);\n",char(var_list(i)),i);
    end
    fprintf(fid,"\n");
    if size(const_list)
        for i=1:max(size(const_list))
            fprintf(fid,"%s = consts(%d);\n",char(const_list(i)),i);
        end
        fprintf(fid,"\n");
    end
    Ndim1 = s(1);
    Ndim2 = s(2);
    if ss==3
        Ndim3 = s(3);
    end
    
    fprintf(fid,"out = zeros(");
    for i=1:ss
        if i<ss
            fprintf(fid,"%d,",s(i));
        else
            fprintf(fid,"%d);\n\n",s(i));
        end
    end
    
    
    %fprintf(fid,"out = zeros(%d,%d);\n\n",Ndim1,Ndim2);
    for i=1:Ndim1
        for j=1:Ndim2
            if ss==3
                for k=1:Ndim3
                    fprintf(fid,"out(%d,%d,%d) = %s;\n",i,j,k,char(expr(i,j,k)));
                end
            else
                fprintf(fid,"out(%d,%d) = %s;\n",i,j,char(expr(i,j)));
            end
        end
        fprintf(fid,"\n");
    end
    fprintf(fid,"end\n");
    fclose(fid);
end
