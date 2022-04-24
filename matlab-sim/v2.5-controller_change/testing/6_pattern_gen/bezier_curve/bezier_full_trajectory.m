function [x,xp,xpp] = bezier_full_trajectory(Pts,tvec)
    x = bezier_curve(Pts,tvec);
    xp = bezier_first_derivative(Pts,tvec);
    xpp = bezier_second_derivative(Pts,tvec);
end

