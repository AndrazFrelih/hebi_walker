function urdf_matlab2ros
    orgFilName = 'florenceRBDL.urdf';
    
    fid = fopen(orgFilName, "r");
    fs = fread(fid);
    fclose(fid);
    
    str1src = '## WORLD';
    str1 = '<link name="World"></link>';
    str2src = '## FLOJOINT';
    %str2 = '<joint name="FloBaseJoint" type="fixed"><origin xyz="0 0 0.6624" rpy="0 0 0"/><parent link="World"/><child link="FloBase"/></joint>';
    str2 = '<joint name="FloBaseJoint" type="fixed"><origin xyz="0 0 0.0" rpy="0 0 0"/><parent link="World"/><child link="FloBase"/></joint>';
    
    fs = strrep(fs,convertStringsToChars(str1src),convertStringsToChars(str1));
    fs = strrep(fs,convertStringsToChars(str2src),convertStringsToChars(str2));
    
    sfilename = 'florenceMatlab.urdf';
    
    fid = fopen(strcat('./urdf/',sfilename),'w');
    fwrite(fid,fs);
    fclose(fid);
    
    
    gfilename = 'florenceROS.urdf';
    gfilenameCmd = 'florenceROScmd.urdf';
    
    init_line = '<?xml version="1.0"?>';
    init_line = init_line';
    
    mnames = mesh_names;
    
    ros_pck_prefix = 'package://florence_description/meshes/';
    
    fid = fopen(sfilename, "r");
    fs = fread(fid);
    fclose(fid);
    
    fs = [double(init_line); 10; fs]; %10 is there for line break
    
    for i=1:max(size(mnames))
        str = strcat(mnames(i),'_rem');
        fs = strrep(fs,convertStringsToChars(str),convertStringsToChars(strcat(ros_pck_prefix,str)));
    end
    
    % this is not working and should notbe used
    %fs = strrep(fs,'<joint name="FloBaseJoint" type="fixed">','<joint name="FloBaseJoint" type="floating">');
    
    fid = fopen(strcat('./urdf/',gfilename),'w');
    fwrite(fid,fs);
    fclose(fid);
    
    %change color
    fs = strrep(fs,'<color rgba="0 1.0 1.0 1.0"/>','<color rgba="1.0 1.0 1.0 0.2"/>');
    
    fid = fopen(strcat('./urdf/',gfilenameCmd),'w');
    fwrite(fid,fs);
    fclose(fid);
    
end

