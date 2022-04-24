Naccel = 3;
Nsens = 16;
eul_true = rand(Naccel,1);
R_true = eul2rotm(eul_true');
n_sc = 10;
n_meas = 2*(rand(Naccel,Nsens)-1/2)/n_sc;
eul_meas = zeros(Naccel,Nsens);

w = ones(Nsens,1);
w = w/sum(w);

Ri = cell(Nsens,1);

for i=1:Nsens
    eul_meas(:,i) = eul_true+n_meas(:,i);
    Ri{i} = eul2rotm(eul_meas(:,i)');
end

R_est = zeros(3,3);
eul_est = zeros(3,1);
for i=1:Nsens
    R_est = R_est + w(i)*Ri{i};
    eul_est = eul_est + w(i)*eul_meas(:,i);
end

R_est_eul = eul2rotm(eul_est');

disp("Determinant of the naively determined rotm: " +det(R_est));
disp("Determinant of the rotm from euler angles: " +det(R_est_eul));

err_est = sum(abs(R_true-R_est),[1 2]);
err_est_eul = sum(abs(R_true-R_est_eul),[1 2]);
disp("Total manhattan error of the naively determined rotm: " +err_est);
disp("Total manhattan error of the rotm from euler angles: " +err_est_eul);
err_sing = zeros(1,Nsens);
for i=1:Nsens
    err_sing(i) = sum(abs(R_true-Ri{i}),[1 2]);
    disp("Error of the single ("+i+"th) measurement " +err_sing(i));
end

