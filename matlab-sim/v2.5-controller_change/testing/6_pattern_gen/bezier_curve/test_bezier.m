Pts = [
    0,0;
    0,0;
    0,0;
    0,1;
    1,1;
    1,0;
    1,0;
    1,0
];

Pts = Pts';

tvec = linspace(0,1,100); % t has to be normalised between 0 and 1

x = bezier_curve(Pts,tvec);
xp = bezier_first_derivative(Pts,tvec);
xpp = bezier_second_derivative(Pts,tvec);

clf;

figure(1);
plot(x(1,:),x(2,:));
grid on;
legend("y(x)");

figure(2);
plot(tvec,x(1,:));
hold on;
plot(tvec,x(2,:));
hold off;
grid on;
legend("x(t)","y(t)");

figure(3);
plot(tvec,xp(1,:));
hold on;
plot(tvec,xp(2,:));
hold off;
grid on;
legend("xp(t)","yp(t)");

figure(4);
plot(tvec,xpp(1,:));
hold on;
plot(tvec,xpp(2,:));
hold off;
grid on;
legend("xpp(t)","ypp(t)");