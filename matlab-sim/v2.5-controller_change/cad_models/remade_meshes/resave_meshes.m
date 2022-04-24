%Low resolution STL file names
STL_file_list = mesh_names;

%file extension
ext = ".stl";

%Number of the files to process
N_stl = max(size(STL_file_list));

%Path to scaled STL files
save_pwd = "./cad_models/remade_meshes/";

%Scaling factor
S = 1000;

for i=1:N_stl
    mesh = stlread(strcat(STL_file_list(i),ext));
    mesh = triangulation(mesh.ConnectivityList,mesh.Points/S);
    path = save_pwd+STL_file_list(i)+"_rem"+ext;
    stlwrite(mesh,path);
end