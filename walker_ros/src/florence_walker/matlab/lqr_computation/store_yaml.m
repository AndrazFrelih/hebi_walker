clc;clear;
addpath(genpath("yaml"));

FID = fopen("../../launch/config/yaml/florence_walker.yaml", "r");
data2read = fread(FID,'*char');
fclose(FID);

data2read = convertCharsToStrings(data2read);
Xread = YAML.load(data2read);
disp(Xread)

% X = struct('matrix', rand(3,4), 'char', 'hello');
% S = YAML.dump(X);
% disp(S);

Ts = 1/Xread.ctrl.full_controller.publish_rate;
zdes = Xread.tgen.zdes;
Nprev = Xread.tgen.Nprev;
[Pctrl] = preview_for_ros(zdes, Ts, Nprev);

Klqr.Kint = Pctrl.Gi;
Klqr.Kcom = Pctrl.Gx;
Klqr.Kref = Pctrl.Gp;
data_structure.tgen.lqr = Klqr;

Xwrite = YAML.dump(data_structure);
data2write = convertStringsToChars(Xwrite);

FID = fopen("../../launch/config/yaml/Klqr.yaml", "w");
fwrite(FID,data2write);
fclose(FID);

macro_def = "#define NPREV "+string(Nprev);
FID = fopen("../../include/preview_settings.h", "w");
fwrite(FID,convertStringsToChars(macro_def));
fclose(FID);