function createGetter(name,list)
    str = "./symbolic_functions/"+name+".m";
    s = size(list);
    ss = max(s);
    fid = fopen(str,"w");
    fprintf(fid,"function [out] = %s\n",char(name));
    fprintf(fid,"\n");
    fprintf(fid,"out = strings(%d,1);\n",ss);
    %fprintf(fid,"out = zeros(%d,%d);\n\n",Ndim1,Ndim2);
    for i=1:ss
        fprintf(fid,"out(%d) = ""%s"";\n",i,list(i));
    end
    fprintf(fid,"end\n");
    fclose(fid);
end