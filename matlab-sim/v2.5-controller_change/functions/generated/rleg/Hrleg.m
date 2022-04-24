function [out] = Hrleg(inp)
qR1 = inp(1);
qR2 = inp(2);
qR3 = inp(3);
qR4 = inp(4);
qR5 = inp(5);
qR6 = inp(6);

out = zeros(6,4,4);

out(1,1,1) = 0.000000000000000061232339957279221655641939602547*cos(qR1);
out(1,1,2) = -1.0*sin(qR1);
out(1,1,3) = -1.0*cos(qR1);
out(1,1,4) = -0.037500000000022737367544323205948*cos(qR1);
out(1,2,1) = 0.000000000000000061232339957279221655641939602547*sin(qR1);
out(1,2,2) = cos(qR1);
out(1,2,3) = -1.0*sin(qR1);
out(1,2,4) = -0.037500000000022737367544323205948*sin(qR1);
out(1,3,1) = 1.0;
out(1,3,2) = 0;
out(1,3,3) = 0.000000000000000061232339957279221655641939602547;
out(1,3,4) = 0.10005000000001018634065985679626;
out(1,4,1) = 0;
out(1,4,2) = 0;
out(1,4,3) = 0;
out(1,4,4) = 1.0;

out(2,1,1) = cos(qR2);
out(2,1,2) = -0.000000000000000061232339957279221655641939602547*sin(qR2);
out(2,1,3) = -1.0*sin(qR2);
out(2,1,4) = 0;
out(2,2,1) = sin(qR2);
out(2,2,2) = 0.000000000000000061232339957279221655641939602547*cos(qR2);
out(2,2,3) = cos(qR2);
out(2,2,4) = 0;
out(2,3,1) = 0;
out(2,3,2) = -1.0;
out(2,3,3) = 0.000000000000000061232339957279221655641939602547;
out(2,3,4) = 0.10005000000001018634065985679626;
out(2,4,1) = 0;
out(2,4,2) = 0;
out(2,4,3) = 0;
out(2,4,4) = 1.0;

out(3,1,1) = cos(qR3);
out(3,1,2) = -1.0*sin(qR3);
out(3,1,3) = 0;
out(3,1,4) = 0.32549999999991996446624398231506*cos(qR3);
out(3,2,1) = sin(qR3);
out(3,2,2) = cos(qR3);
out(3,2,3) = 0;
out(3,2,4) = 0.32549999999991996446624398231506*sin(qR3);
out(3,3,1) = 0;
out(3,3,2) = 0;
out(3,3,3) = 1.0;
out(3,3,4) = 0;
out(3,4,1) = 0;
out(3,4,2) = 0;
out(3,4,3) = 0;
out(3,4,4) = 1.0;

out(4,1,1) = cos(qR4);
out(4,1,2) = sin(qR4);
out(4,1,3) = 0.00000000000000012246467991455844331128387920509*sin(qR4);
out(4,1,4) = 0.32499999999981810105964541435242*cos(qR4);
out(4,2,1) = sin(qR4);
out(4,2,2) = -1.0*cos(qR4);
out(4,2,3) = -0.00000000000000012246467991455844331128387920509*cos(qR4);
out(4,2,4) = 0.32499999999981810105964541435242*sin(qR4);
out(4,3,1) = 0;
out(4,3,2) = 0.00000000000000012246467991455844331128387920509;
out(4,3,3) = -1.0;
out(4,3,4) = -0.019999999999981810105964541435242;
out(4,4,1) = 0;
out(4,4,2) = 0;
out(4,4,3) = 0;
out(4,4,4) = 1.0;

out(5,1,1) = cos(qR5);
out(5,1,2) = -0.000000000000000061232339957279221655641939602547*sin(qR5);
out(5,1,3) = -1.0*sin(qR5);
out(5,1,4) = 0.023000000000024556356947869062424*sin(qR5);
out(5,2,1) = sin(qR5);
out(5,2,2) = 0.000000000000000061232339957279221655641939602547*cos(qR5);
out(5,2,3) = cos(qR5);
out(5,2,4) = -0.023000000000024556356947869062424*cos(qR5);
out(5,3,1) = 0;
out(5,3,2) = -1.0;
out(5,3,3) = 0.000000000000000061232339957279221655641939602547;
out(5,3,4) = 0.10005000000001018634065985679626;
out(5,4,1) = 0;
out(5,4,2) = 0;
out(5,4,3) = 0;
out(5,4,4) = 1.0;

out(6,1,1) = - 0.000000000000000061232339957279221655641939602547*cos(qR6) - 0.00000000000000012246467991455844331128387920509*sin(qR6);
out(6,1,2) = sin(qR6) - 7.4987989132990300965134877452522e-33*cos(qR6);
out(6,1,3) = -1.0*cos(qR6);
out(6,1,4) = 0.11917500000004110916052013635635*cos(qR6);
out(6,2,1) = 0.00000000000000012246467991455844331128387920509*cos(qR6) - 0.000000000000000061232339957279221655641939602547*sin(qR6);
out(6,2,2) = - 1.0*cos(qR6) - 7.4987989132990300965134877452522e-33*sin(qR6);
out(6,2,3) = -1.0*sin(qR6);
out(6,2,4) = 0.11917500000004110916052013635635*sin(qR6);
out(6,3,1) = -1.0;
out(6,3,2) = -0.00000000000000012246467991455844331128387920509;
out(6,3,3) = 0.000000000000000061232339957279221655641939602547;
out(6,3,4) = 0.033050000000002910383045673370361;
out(6,4,1) = 0;
out(6,4,2) = 0;
out(6,4,3) = 0;
out(6,4,4) = 1.0;

end
