clear;
clc;

m1 = 0.718;
m2 = 0.419;
l1 = 0.22;
l2 = 0.26;
len = 0.13:0.01:0.4;

ds = 0.23;
F = 250;

fprintf('%.8f\n',atan2(3,11));
for i = 1:length(len)
    l = len(i);
    alpha1 = acos((l^2+l1^2-l2^2)/(2*l*l1));
    phi1 = pi/2+alpha1;
    phi4 = pi/2-alpha1;
    x1 = l1*cos(phi1);
    y1 = l1*sin(phi1);
    alpha2 = acos((l1^2+l2^2-l^2)/(2*l1*l2));
    phi2 = alpha2-(pi-phi1);
    x2 = 0;
    y2 = l;
    x1com = x1/2;
    y1com = y1/2;
    x2com = (x1+x2)/2;
    y2com = (y1+y2)/2;
    xcom = (m1*x1com+m2*x2com)/(m1+m2);
    ycom = (m1*y1com+m2*y2com)/(m1+m2);
    dl = sqrt(xcom^2+ycom^2);
    thetal0 = atan2(-xcom,ycom);
    Ic1 = (1/12)*m1*l1^2 + m1*(l1/2)^2;
    Ic2 = (1/12)*m2*l2^2 + m2*(( (x1+x2)/2 )^2 + ( (y1+y2)/2 )^2);
    Il = Ic1+Ic2;
    fprintf('%.2f, %.8f, %.8f, %.8f;\n', l, thetal0, dl, Il);
    A = l1 * sin(phi1 - phi2) * sin(pi/2 - (pi - phi2)) / sin((pi - phi2) - phi2);
    J11 = A;
    J12 = -A;
    J21 = 0.5;
    J22 = 0.5;
    Fcomp = F*cos(alpha1);
    Tcomp = F*sin(phi1-phi2)*ds;
    T1comp = (J11*Fcomp + J21*Tcomp);
    T4comp = (J12*Fcomp + J22*Tcomp);
    fprintf('%.2f, %.8f, %.8f, %.8f, %.8f;\n', l, Fcomp, Tcomp, T1comp, T4comp);
end