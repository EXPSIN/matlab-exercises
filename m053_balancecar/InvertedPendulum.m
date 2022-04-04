clear; close all; clf; clc;

GS = IP_Interface('i', 20e-3, pi-0.5);


for idx = 1:1000
    GS.t = idx*GS.T;
    GS.car.u = controller(GS);
%     if(mod(idx, 100) ==0 )
%         GS.car.u = GS.car.u + 0.1;
%     end
%     GS.car.u = -1.0;
    GS = IP_Interface('u', GS);


%     GS.pen.theta = idx*0.01;
%     GS = IP_Interface('s', GS);
    drawnow;
end


function u = controller(GS)
    % ������1- ״̬����
    u = 50*(pi-GS.pen.theta) + 10 * (0-GS.pen.thetav)...
      - 1*(0-GS.car.p) - 4*(0-GS.car.v);
  
    % Ŀ��������
%     V_aim = 2*GS.pen.mass*GS.g*GS.pen.len + 0;
%     % ��ǰ������
%     V = GS.pen.mass*GS.g*GS.pen.len*(1-cos(GS.pen.theta)) + ...
%         1/2*GS.pen.mass*GS.pen.thetav^2;
%     
%     u = -8*(V_aim - V);
end