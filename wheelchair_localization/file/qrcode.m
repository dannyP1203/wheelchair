clear all
clc
format long
n='';
file=strcat('QRodom',n,'.txt');
fid = fopen(file);
[Xqr, Yqr, Zqr, heading, Bx, By, Bz, Xg, Yg, Zg, t]=textread(file, '%f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

figure
plot(Xg1,Yg1,'k',Xqr,Yqr,'g');
title('XY');legend('Gazebo','QR','Location','northwest');
xlabel('X [m]'); ylabel('Y [m]'); grid on;

figure
subplot(1,2,1);plot(timeukfqr,Xg1,'k',timeqr,Xqr,'g');
title('X');legend('Gazebo','QR','Location','northwest');xlabel('time[s]'); ylabel('X [m]'); grid on;
subplot(1,2,2);plot(timeukfqr,Yg1,'k',timeqr,Yqr,'g');
title('Y');legend('Gazebo','QR','Location','northwest');xlabel('time[s]'); ylabel('Y [m]'); grid on;

Eaqr = [abs(Xqr-Xg), abs(Yqr-Yg)];
Ex=[mean(Eaqr(:,1)), var(Eaqr(:,1))];
Ey=[mean(Eaqr(:,2)), var(Eaqr(:,2))];
MSE = sqrt(sum(((Yqr-Yg).^2) + ((Xqr-Xg).^2)))/length(Xqr);

