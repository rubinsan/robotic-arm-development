% Variables
clear all
clc
l1=0.205;
l2=0.257;
l3=0.265;
l4=0.220;

t=(0:0.05:2);

% Valores iniciales
teta_1=pi; % 90º
teta_2=0.88; % 45º
teta_3=-1.89; % -45º
teta_4=0.99; % 45º

% Cinematica Directa (punto inicial)

x0=0;
y0=0;
z0=0;
p0=[x0 y0 z0 0]';

x1=0;
y1=0;
z1=l1;
p1=[x1 y1 z1 0]';
    
x2=cos(teta_1+pi/2)*(l2*cos(teta_2));
y2=sin(teta_1+pi/2)*(l2*cos(teta_2));
z2=l1+(l2*sin(teta_2));
p2=[x2 y2 z2 0]';

x3=cos(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3)));
y3=sin(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3)));
z3=l1+(l2*sin(teta_2))+(l3*sin(teta_2+teta_3));
p3=[x3 y3 z3 0]';

xe=cos(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
ye=sin(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
ze=(l1)+(l2*sin(teta_2))+(l3*sin(teta_2+teta_3))+(l4*sin(teta_2+teta_3+teta_4));
oe=teta_2+teta_3+teta_4;
pe=[xe ye ze oe]';
    
figure
plot3([x0; x1], [y0; y1], [z0; z1], 'c')
title (['POS INIC-> ', ' Teta1:', num2str(teta_1), '  Teta2:', num2str(teta_2), '  Teta3:', num2str(teta_3), '  Teta4:', num2str(teta_4)])
hold on
plot3([x1; x2], [y1; y2], [z1; z2], 'c')
plot3([x2; x3], [y2; y3], [z2; z3], 'c')
plot3([x3; xe], [y3; ye], [z3; ze], 'c')

center=[0,0];
viscircles(center,0.15,'Color','k');
th = 0:pi/50:2*pi;
h=(ones(1,101)*0.155);
xunit = 0.15 * cos(th);
yunit = 0.15 * sin(th);
h = plot3(xunit,yunit,h,'k');

wq=zeros(4,20);
        

for i=1:40
    
    % Cinemática inversa (obtención de q space)
    % diff(pe)=J*diff(q) -> diff(q)=J^-1*diff(pe)

    J=zeros(4);

    J(1,1)=-sin(teta_1+pi/2)*((l2*cos(teta_2))+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
    J(1,2)=-(sin(teta_2)*cos(teta_1+pi/2)*l2)-(sin(teta_2+teta_3)*cos(teta_1+pi/2)*l3)-(sin(teta_2+teta_3+teta_4)*cos(teta_1+pi/2)*l4);
    J(1,3)=-(sin(teta_2+teta_3)*cos(teta_1+pi/2)*l3)-(sin(teta_2+teta_3+teta_4)*cos(teta_1+pi/2)*l4);
    J(1,4)=-(sin(teta_2+teta_3+teta_4)*cos(teta_1+pi/2)*l4);

    J(2,1)=cos(teta_1+pi/2)*((l2*cos(teta_2))+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
    J(2,2)=-(sin(teta_2)*sin(teta_1+pi/2)*l2)-(sin(teta_2+teta_3)*sin(teta_1+pi/2)*l3)-(sin(teta_2+teta_3+teta_4)*sin(teta_1+pi/2)*l4);
    J(2,3)=-(sin(teta_2+teta_3)*sin(teta_1+pi/2)*l3)-(sin(teta_2+teta_3+teta_4)*sin(teta_1+pi/2)*l4);
    J(2,4)=-(sin(teta_2+teta_3+teta_4)*sin(teta_1+pi/2)*l4);
    
    J(3,1)=0;
    J(3,2)=(l2*cos(teta_2))+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4));
    J(3,3)=(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4));
    J(3,4)=l4*cos(teta_2+teta_3+teta_4);
    
    J(4,1)=0;
    J(4,2)=1;
    J(4,3)=1;
    J(4,4)=1;

    detJ=det(J);
    display(detJ);
    invJ=inv(J);

    % Se impone un movimiento rectilineo en x a 0.3 m/s sin rotación del
    % end effector
    dxe=0; 
    dye=-0.1;
    dze=0;
    dtetae=0;

    diffpe=[dxe dye dze dtetae]'; % velocidad en el end effector (x y z + rot)
    diffq=invJ*diffpe; % obtención de las velocidades articulares
    
    wq(:,i)=diffq;
    
    %incr_teta1=int(diffq(1),t,t(i),t(i+1));
  
    incr_teta_1=diffq(1)*(t(i+1)-t(i)); % Integracion en t
    teta_1=teta_1+incr_teta_1;
    
    incr_teta_2=diffq(2)*(t(i+1)-t(i)); % Integracion en t
    teta_2=teta_2+incr_teta_2;
    
    incr_teta_3=diffq(3)*(t(i+1)-t(i)); % Integracion en t
    teta_3=teta_3+incr_teta_3;
    
    incr_teta_4=diffq(4)*(t(i+1)-t(i)); % Integracion en t
    teta_4=teta_4+incr_teta_4;

    % Cinematica Directa (para representar)

    x0=0;
    y0=0;
    z0=0;
    p0=[x0 y0 z0 0]';

    x1=0;
    y1=0;
    z1=l1;
    p1=[x1 y1 z1 0]';
    
    x2=cos(teta_1+pi/2)*(l2*cos(teta_2));
    y2=sin(teta_1+pi/2)*(l2*cos(teta_2));
    z2=l1+(l2*sin(teta_2));
    p2=[x2 y2 z2 0]';

    x3=cos(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3)));
    y3=sin(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3)));
    z3=l1+(l2*sin(teta_2))+(l3*sin(teta_2+teta_3));
    p3=[x3 y3 z3 0]';

    xe=cos(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
    ye=sin(teta_1+pi/2)*(l2*cos(teta_2)+(l3*cos(teta_2+teta_3))+(l4*cos(teta_2+teta_3+teta_4)));
    ze=(l1)+(l2*sin(teta_2))+(l3*sin(teta_2+teta_3))+(l4*sin(teta_2+teta_3+teta_4));
    oe=teta_2+teta_3+teta_4;
    %display(oe);
    pe=[xe ye ze oe]';
    
    plot3(x0, y0, z0, 'ko')
    plot3(x1, y1, z1, 'go')
    plot3(x2, y2, z2, 'bo')
    plot3(x3, y3, z3, 'mo')
    plot3(xe, ye, ze, 'ro')
    
    plot3([x0; x1], [y0; y1], [z0; z1], 'k')
    plot3([x1; x2], [y1; y2], [z1; z2], 'k')
    plot3([x2; x3], [y2; y3], [z2; z3], 'k')
    plot3([x3; xe], [y3; ye], [z3; ze], 'k')

end

hold off

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

axis([-1,1,-1,1,0,1]);

i=(1:1:40);

figure

plot(i,wq(1,:));
hold on
plot(i,wq(2,:));
plot(i,wq(3,:));
plot(i,wq(4,:));
hold off
legend('qo','q1','q2','q3')
title ('Angular speeds')




