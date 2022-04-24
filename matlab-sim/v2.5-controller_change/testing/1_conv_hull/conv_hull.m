%% Base
%base stl file
base_stl = stlread("Base.stl");
pts = base_stl.Points;
con_mat_1 = base_stl.ConnectivityList;
figure(1);
trisurf(con_mat_1,pts(:,1),pts(:,2),pts(:,3),'FaceColor','blue');
axis equal;

%first convex hull
[con_mat_2,av1] = convhull(pts,'Simplify',false);
figure(2);
trisurf(con_mat_2,pts(:,1),pts(:,2),pts(:,3),'FaceColor','blue');
axis equal;

%this convex hull should have less points
[con_mat_3,av2] = convhull(pts,'Simplify',true);
figure(3);
trisurf(con_mat_3,pts(:,1),pts(:,2),pts(:,3),'FaceColor','blue');
axis equal;

%% Left and right links
%l&r first link stl file
link_1L = stlread("Link1L.stl");
pts_1L = link_1L.Points;
con_mat_1L_1 = link_1L.ConnectivityList;

link_1R = stlread("Link1R.stl");
pts_1R = link_1R.Points;
con_mat_1R_1 = link_1R.ConnectivityList;

figure(1)
hold on;
trisurf(con_mat_1L_1,pts_1L(:,1),pts_1L(:,2),pts_1L(:,3),'FaceColor','red');
trisurf(con_mat_1R_1,pts_1R(:,1),pts_1R(:,2),pts_1R(:,3),'FaceColor','red');
hold off;

%compute a simplified convex hull for both links
[con_mat_1L_3,av_1L_3] = convhull(pts_1L,'Simplify',true);
[con_mat_1R_3,av_1R_3] = convhull(pts_1R,'Simplify',true);

figure(3)
hold on;
trisurf(con_mat_1L_3,pts_1L(:,1),pts_1L(:,2),pts_1L(:,3),'FaceColor','red');
trisurf(con_mat_1R_3,pts_1R(:,1),pts_1R(:,2),pts_1R(:,3),'FaceColor','red');
hold off;