function [Px, Py, Pz] = MyCobot600_FK_fun(t1,t2,t3,t4,t5,t6)
    % Cobot Values 
    a1 = 210;
    a2 = 250;
    a3 = 250;
    a4 = 109.5;
    a5 = 107; 
    a6 = 76.2;
    % deg to rad conversion 
    t1 = t1*pi/180; t2 = t2*pi/180; t3 = t3*pi/180; t4 = t4*pi/180; t5 = t5*pi/180; 
    %t6 = t6*pi/180;
    
    % X/Y/Z position from Transformation Matrix
    Px = a2*cos(t1)*cos(t2) - a4*sin(t1) - a6*cos(t5)*sin(t1) - a5*sin(t2 + t3 + t4)*cos(t1) + a3*cos(t1)*cos(t2)*cos(t3) - a3*cos(t1)*sin(t2)*sin(t3) + a6*cos(t2 + t3 + t4)*cos(t1)*sin(t5);   
    Py = a6*(cos(t1)*cos(t5) + cos(t2 + t3 + t4)*sin(t1)*sin(t5)) + a4*cos(t1) + a2*cos(t2)*sin(t1) - a5*sin(t2 + t3 + t4)*sin(t1) + a3*cos(t2)*cos(t3)*sin(t1) - a3*sin(t1)*sin(t2)*sin(t3);
    Pz = a1 + a5*(sin(t2 + t3)*sin(t4) - cos(t2 + t3)*cos(t4)) - a3*sin(t2 + t3) - a2*sin(t2) - a6*sin(t5)*(cos(t2 + t3)*sin(t4) + sin(t2 + t3)*cos(t4));
