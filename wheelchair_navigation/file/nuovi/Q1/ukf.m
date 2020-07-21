clear all
clc
format long
n='';
file = strcat('UKFodom',n,'.txt');
fid = fopen(file);
[Xukf, Yukf, Xi, Yi, Xe, Ye, Xg, Yg, Hukf, t]=textread(file, '%f %f %f %f %f %f %f %f %f %f');
fclose(fid);

figure
plot(Xg,Yg,'k');hold on;plot(Xe,Ye,'b'); plot(Xi,Yi,'g'); plot(Xukf,Yukf,'r'); 
title('XY');legend('GAZ','Enc','Enc+IMU','UKF','Location','northwest');xlabel('X [m]'); ylabel('Y [m]');grid on;
figure
subplot(1,2,1); plot(t,Xg,'k'); hold on; plot(t,Xe,'b');plot(t,Xi,'g');plot(t,Xukf,'r');
title('X');legend('GAZ','Enc','Enc+IMU','UKF','Location','northwest');xlabel('time [s]'); ylabel('X [m]'); grid on;
subplot(1,2,2); plot(t,Yg,'k'); hold on; plot(t,Ye,'b');plot(t,Yi,'g');plot(t,Yukf,'r');
title('Y');legend('GAZ','Enc','Enc+IMU','UKF','Location','northwest');xlabel('time [s]'); ylabel('Y [m]'); grid on;

% calcoliamo l'errore Enc, Enc+IMU, UKF
EaE = [abs(Xe-Xg), abs(Ye-Yg)];
EaI = [abs(Xi-Xg), abs(Yi-Yg)];
EaUKF = [abs(Xukf-Xg), abs(Yukf-Yg)];

% MEDIA E VARIANZA DELL'ERRORE
Ex=[mean(EaE(:,1)), var(EaE(:,1));...
    mean(EaI(:,1)), var(EaI(:,1));..., mean(Erukf(:,1)), var(Erukf(:,1));...
    mean(EaUKF(:,1)), var(EaUKF(:,1))]%, mean(Erukfqr(:,1)), var(Erukfqr(:,1))]
Ey=[mean(EaE(:,1)), var(EaE(:,1));...
    mean(EaI(:,2)), var(EaI(:,2));..., mean(Erukf(:,2)), var(Erukf(:,2));...
    mean(EaUKF(:,2)), var(EaUKF(:,2))]%, mean(Erukfqr(:,2)), var(Erukfqr(:,2))]

% ERRORE QUADRATICO MEDIO
MSE = [sqrt(sum(((Ye-Yg).^2) + ((Xe-Xg).^2)))/length(Xukf);...
       sqrt(sum(((Yi-Yg).^2) + ((Xi-Xg).^2)))/length(Xukf);...
       sqrt(sum(((Yukf-Yg).^2) + ((Xukf-Xg).^2)))/length(Xukf)]
   
figure
dist = [MSE(1) MSE(2) MSE(3)];
bar(dist)
ylabel('Errore Quadratico Medio [m]');
X=[1,2,3];
labels = arrayfun(@(value) num2str(value,'%2.22f'),dist,'UniformOutput',false);
text(X,dist,labels,'HorizontalAlignment','center','VerticalAlignment','bottom') 
set(gca,'xticklabel',{'MSE Enc','MSE Enc+IMU','MSE UKF'});

