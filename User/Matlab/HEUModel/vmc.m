syms phi0 phi1 phi2 phi3 phi4 L1 L2 L
A = L1*(sin(phi0-phi1));
B = L2*(sin(phi0-phi2));
C = L2*(sin(phi0-phi3));
D = L1*(sin(phi0-phi4));

E = L1*(cos(phi0-phi1));
F = L2*(cos(phi0-phi2));
G = L2*(cos(phi0-phi3));
H = L1*(cos(phi0-phi4));

K12 = (L1*sin(phi3-phi1))/(L2*sin(phi2-phi3));
K13 = (L1*sin(phi2-phi1))/(L2*sin(phi2-phi3));
K42 = (L1*sin(phi4-phi3))/(L2*sin(phi2-phi3));
K43 = (L1*sin(phi4-phi2))/(L2*sin(phi2-phi3));

d31 = (L1*sin(phi1-phi4))/(L2*sin(phi3-phi4));
d41 = (L1*sin(phi1-phi3))/(L2*sin(phi4-phi3));
d32 = sin(phi2-phi4)/sin(phi3-phi4);
d42 = sin(phi2-phi3)/sin(phi4-phi3);

% L0 = L1*(cos(phi0-phi1)) + L2*(cos(phi0-phi2)) + L2*(cos(phi0-phi3)) + L1*(cos(phi0-phi4));

J = [A+B*K12+C*K13, D+B*K42+C*K43;(E+F*K12+G*K13)/L, (H+F*K42+G*K43)/L];
simplify(J)

L1 = 0.15;
L2 = 0.27;
L3 = 0.15;

xb = L1*cos(phi1) - L3/2;
yb = L1*sin(phi1);
xd = L1*cos(phi4) + L3/2;
yd = L1*sin(phi4);

A0 = 2*L2*(xd-xb);
B0 = 2*L2*(yd-yb);
C0 = (xd-xb)^2 + (yd-yb)^2;

phi2 = 2*atan2((B0+sqrt(A0^2+B0^2-C0^2)),(A0+C0));

xc = xb + L2*cos(phi2);
yc = yb + L2*sin(phi2);

phi3 = atan2(yd-yc, xd-xc)+pi;

L = sqrt(xc^2 + yc^2);
phi0 = atan2(yc, xc);
% J = jacobian([L,phi0],[phi1,phi4]);
J = subs(J);
syms F Tp;
T = J.'*[F;Tp];
% matlabFunction(T,'File','vmcT');
% syms phi1dot phi4dot;
% xdot = J*[phi1dot;phi4dot];
% matlabFunction(L,phi0,xdot,'File','vmcX');
