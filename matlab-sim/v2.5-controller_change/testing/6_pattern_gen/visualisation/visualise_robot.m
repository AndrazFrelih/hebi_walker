function visualise_robot(robot_model,Qalg,ang)
    Qrt = alg_to_rtbox(Qalg);
    robot_model.DataFormat = 'col';
    show(robot_model,Qrt);
    view([ang 0]);
    camproj('orthographic');
    axis equal;
    grid on;
end

