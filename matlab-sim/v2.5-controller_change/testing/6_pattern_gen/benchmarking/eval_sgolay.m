%% TEST: does it make sense to filter distance or velocity datapoints?
v1 = [zeros(Nz,1), (pos(:,2:end)-pos(:,1:end-1))/Tsample];
posfil = zeros(size(pos));
v2 = zeros(size(pos));
for i = 1 : size(pos,1)
    posfil(i,:) = sgolayfilt(pos(i,:),5,11);
    v2(i,:) = sgolayfilt(v1(i,:),5,11);
end
v3 = [zeros(Nz,1), (posfil(:,2:end)-posfil(:,1:end-1))/Tsample];

ind_2_plt = 4;
figure(24);
hold on
plot(v1(ind_2_plt,:));
plot(v2(ind_2_plt,:));
plot(v3(ind_2_plt,:));
grid on
legend(["VELO: Not filt.","VELO: Filt. vel. vect.", "VELO: Filt. pos. vect."]);

%% RESULT: both v2 and v3 are exactly the same. 
% SGolay filter therefore performs a linear operation on the data.
% Filtering before does not make any difference.