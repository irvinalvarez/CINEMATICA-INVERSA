clear all
close all
clc

%% Lectura de distancias
prompt = 'Introduza la longitud del primer eslabon (l1): ';
l1 = input(prompt);
prompt = 'Introduza la longitud del primer eslabon (l2): ';
l2 = input(prompt);
prompt = 'Introduza la longitud del primer eslabon (l3): ';
l3 = input(prompt);

%% Lectura de punto final
prompt = 'Introduza el punto sobre el eje x: ';
x = input(prompt);
prompt = 'Introduza el punto sobre el eje y: ';
y = input(prompt);
prompt = 'Introduza el punto sobre el eje z: ';
z = input(prompt);


%% Definición de los parámetros de Denavit-Hartenberg
d1 = l1;
d2 = 0;
d3 = 0;

a1 = 0;
a2 = l2;
a3 = l3;

alpha_1 = 90;
alpha_2 = 0;
alpha_3 = 0;

alpha_11_rad = deg2rad(alpha_1);
alpha_22_rad = deg2rad(alpha_2);
alpha_33_rad = deg2rad(alpha_3);


%%%% defincion de los angulos del robot
%% Definición del punto inicial del robot
p1 = [0,0,0];


custheta3 = (x^2+y^2+(z-l1)^2-(l2)^2-(l3)^2)/(2*(l2)*(l3));

% theta 1

theta1 = atan2 (y,x);


% theta 3

x3= atan2 ((sqrt(1-(custheta3)^2)),(custheta3));
 
theta3 = -x3;

b = atan2((z-l1),(x^2+y^2));

a = atan2 ((l3*(1-custheta3)),(l2+l3*custheta3));


%theta 2

theta2 = (b+a);



if theta1>=0
    angulo = 0:0.01:theta1;
else
    angulo = 0:-0.01:theta1;
end

for i=1:length(angulo)
    
    clf
    
    printAxis();
    grid on 
    Rotz = [cos(angulo(i)) -sin(angulo(i))  0; sin(angulo(i)) cos(angulo(i)) 0; 0 0 1];
    A1 = dhParameters(angulo(i),d1,a1,alpha_11_rad);
    A2 = dhParameters(0,d2,a2,alpha_22_rad);
    A3 = dhParameters(0,d3,a3,alpha_33_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end
    pause(1);
    
    
if theta2>=0
    angulo = 0:0.01:theta2;
else
    angulo = 0:-0.01:theta2;
end

for i=1:length(angulo)
    clf
    printAxis();
    grid on 
    Rotz = [cos(angulo(i)) -sin(angulo(i))  0; sin(angulo(i)) cos(angulo(i)) 0; 0 0 1];
    A1 = dhParameters(theta1,d1,a1,alpha_11_rad);
    A2 = dhParameters(angulo(i),d2,a2,alpha_22_rad);
    A3 = dhParameters(0,d3,a3,alpha_33_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end

pause(1);

if theta3>=0
    angulo = 0:0.01:theta3;
else
    angulo = 0:-0.01:theta3;
end

for i=1:length(angulo)
    clf
    printAxis();
    grid on 
    Rotz = [cos(angulo(i)) -sin(angulo(i))  0; sin(angulo(i)) cos(angulo(i)) 0; 0 0 1];
    A1 = dhParameters(theta1,d1,a1,alpha_11_rad);
    A2 = dhParameters(theta2,d2,a2,alpha_22_rad);
    A3 = dhParameters(angulo(i),d3,a3,alpha_33_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    
 
    view(30,30);
    pause(0.01);
end
