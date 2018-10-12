clc; clear all; close all;

format long g

m = 0.5*2;
L = 0.2;
g = -9.80665;

t = 0;
t_end = 3;
t_old = 0;
h = 0.001;

pos = pi/2;
vel = 0;
acc = 0;

intcount = 1;

r_hat = 0;
K = 300;

des_vel = 0.5;
des_vel_origin = 0.5;
err_vel = 0;
err_vel_prev = 0;
err_vel_accum = 0;
Kp_vel = 3;
Ki_vel = 0;
Kd_vel = 0;

index = 1;
data(index,:) = [t, pos*180/pi, vel*180/pi, acc*180/pi, r_hat];

y = [pos;vel;r_hat];
y_old = y;
yp_old = zeros(3,1);

delta_w = des_vel_origin;
t1 = 0.5;
a = -2*delta_w/t1^3;
b = 3*delta_w/t1^2;

err_vel_prev2 = 0;
d_time = 0;
d_vel = 0;
d_flag = 0;

t_inter_start = 1.0;
t_inter_end = 1.05;

while(t <= t_end)
    pos = y(1);
    vel = y(2);
    
    if t <= t1
        des_vel = a*t^3 + b*t^2;
    end
    
    if t > t1 && t > t_inter_end && t <= t_inter_end + t1
        if d_flag == 0
            delta_w = des_vel_origin - vel;
            d_time = t;
            d_vel = vel;
            a = -2*delta_w/t1^3;
            b = 3*delta_w/t1^2;
            d_flag = 1;
        end
        des_vel = a*(t - d_time)^3 + b*(t - d_time)^2 + d_vel;
    end
    
    err_vel = des_vel - vel;
    err_vel_accum = err_vel_accum + err_vel*h;
    T_control_vel = Kp_vel*err_vel + Kd_vel*(err_vel - err_vel_prev)/h + Ki_vel*err_vel_accum;
    err_vel_prev = err_vel;
    T_in = T_control_vel;

    Tg = -m*g*L*sin(pos);
    Tc = T_in + Tg;

    if (t >= t_inter_start && t <= t_inter_end)
        Td = -1.3;
    else
        Td = 0;
    end
%     Td = 0;
    
    acc = (m*g*L*sin(pos) + Tc + Td)/(m*L^2);
    
    g_q = m*g*L*sin(pos);
    p = 0.5*m*L^2*vel^2;
    
    r_hat_dot = Tc + g_q - r_hat;
    
    yp = [vel;acc;r_hat_dot];
    
%     [y, t, intcount] = absh3( t, y, yp, h, intcount);
    
    y = y_old + yp_old*h + 0.5*h*(yp - yp_old);
    y_old = y;
    yp_old = yp;
    t = t + h;
    
    r_hat = K*(y(3) - p);

    disp([t, pos*180/pi, vel*180/pi, acc*180/pi, r_hat, err_vel])
    index = index + 1;
    data(index,:) = [t, pos*180/pi, vel*180/pi, acc*180/pi, r_hat];
end

% save('dis','data');

figure
set(gcf,'color',[1,1,1])
subplot(131)
plot(data(:,1), data(:,2),'LineWidth',2)
grid on
xlabel('Time [s]','FontSize',13)
ylabel('Position [deg]','FontSize',13)

subplot(132)
plot(data(:,1), data(:,3),'LineWidth',2)
grid on
xlabel('Time [s]','FontSize',13)
ylabel('Velocity [deg/s]','FontSize',13)

subplot(133)
plot(data(:,1), data(:,5),'LineWidth',2)
grid on
xlabel('Time [s]','FontSize',13)
ylabel('residual [Nm]','FontSize',13)

% dis_data = load('dis');
% nodis_data = load('nodis');

% figure
% set(gcf,'color',[1,1,1])
% plot(nodis_data.data(:,1), nodis_data.data(:,5) - dis_data.data(1:size(nodis_data.data, 1), 5), 'LineWidth',2)
% grid on

% subplot(412)
% plot(data(:,1), data(:,3),'LineWidth',2)
% grid on
% xlabel('Time [s]','FontSize',13)
% ylabel('Velocity [deg/s]','FontSize',13)

% subplot(414)
% plot(data(:,1), data(:,6),'LineWidth',2)
% grid on
% xlabel('Time [s]','FontSize',13)
% ylabel('T [Nm]','FontSize',13)

% %% animation
% % Video file open
% makeVideo = VideoWriter('SinglePendulumSimpleModel');
% % Frame Rate - 숫자가 클 수록 재생 속도가 빠름
% makeVideo.FrameRate = 10;
% % Quality - 용량과 관련 됨 (0 ~ 100)
% makeVideo.Quality = 80;
% open(makeVideo)
% 
% i = 1;
% length = size(data,1);
% figure
% set(gcf,'Color',[1,1,1])
% for i = 1 : 50 : length
%     theta = data(i,2)*pi/180;
%     x = L*sin(theta);
%     y = -L*cos(theta);
%     
%     plot(x, y,'o','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k')
%     hold on
%     plot([0,x],[0,y],'k','LineWidth',2)
%     hold off
%     grid on
%     axis([0, L+0.1, 0, 0.05])
%     axis equal
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     
%     pause(0.000000001);
%     disp(i);
%     
%     frame = getframe(gcf);
%     writeVideo(makeVideo,frame);
% end
% 
% close(makeVideo);