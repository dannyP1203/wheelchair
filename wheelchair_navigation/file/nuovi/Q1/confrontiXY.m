clear all
clc
format long
n='';
file = strcat('QRodom',n,'.txt');
fid = fopen(file);
[Xqr, Yqr, Zqr, Hqr, Bx, By, Bz, Xg, Yg, Zg, timeqr]=textread(file, '%f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);
tqr=1:1:length(Xqr);
file = strcat('UKFodom',n,'.txt');
fid = fopen(file);
[Xukf, Yukf, Xi, Yi, Xe, Ye, Xg1, Yg1, Hukf, timeukf]=textread(file, '%f %f %f %f %f %f %f %f %f %f');
fclose(fid);
tukf=1:1:length(Xukf);
file = strcat('UKFQRodom',n,'.txt');
fid = fopen(file);
[Xukfqr, Yukfqr, Xi2, Yi2, Xe2, Ye2, Xg2, Yg2, Hukfqr, timeukfqr]=textread(file, '%f %f %f %f %f %f %f %f %f %f');
fclose(fid);

figure
subplot(1,2,1);plot(timeukf,Xg1,'k',timeqr,Xqr,'g',timeukf,Xukf,'b',timeukfqr,Xukfqr,'r');
title('X');legend('Gazebo','QR','UKF','UKFQR','Location','northwest');xlabel('time[s]'); ylabel('X [m]'); grid on;
subplot(1,2,2);plot(timeukf,Yg1,'k',timeqr,Yqr,'g',timeukf,Yukf,'b',timeukfqr,Yukfqr,'r');
title('Y');legend('Gazebo','QR','UKF','UKFQR','Location','northwest');xlabel('time[s]'); ylabel('Y [m]'); grid on;

% calcoliamo l'errore QRcode, UKFcode, UKFQRcode
Eaqr = [abs(Xqr-Xg), abs(Yqr-Yg)];
Eaukf = [abs(Xukf-Xg1), abs(Yukf-Yg1)];
Eaukfqr = [abs(Xukfqr-Xg2), abs(Yukfqr-Yg2)];

% MEDIA E VARIANZA DELL'ERRORE
Ex=[mean(Eaqr(:,1)), var(Eaqr(:,1));...
    mean(Eaukf(:,1)), var(Eaukf(:,1));..., mean(Erukf(:,1)), var(Erukf(:,1));...
    mean(Eaukfqr(:,1)), var(Eaukfqr(:,1))]%, mean(Erukfqr(:,1)), var(Erukfqr(:,1))]
Ey=[mean(Eaqr(:,1)), var(Eaqr(:,1));...
    mean(Eaukf(:,2)), var(Eaukf(:,2));..., mean(Erukf(:,2)), var(Erukf(:,2));...
    mean(Eaukfqr(:,2)), var(Eaukfqr(:,2))]%, mean(Erukfqr(:,2)), var(Erukfqr(:,2))]

% ERRORE QUADRATICO MEDIO
MSE = [sqrt(sum(((Yqr-Yg).^2) + ((Xqr-Xg).^2)))/length(Xqr);...
       sqrt(sum(((Yukf-Yg1).^2) + ((Xukf-Xg1).^2)))/length(Xukf);...
       sqrt(sum(((Yukfqr-Yg2).^2) + ((Xukfqr-Xg2).^2)))/length(Xukfqr)]

figure
plot(Xg1,Yg1,'k',Xqr,Yqr,'g',Xukf,Yukf,'b',Xukfqr,Yukfqr,'r');
title('XY');legend('Gazebo','QR','UKF','UKFQR','Location','northwest');
xlabel('X [m]'); ylabel('Y [m]'); grid on;

figure
subplot(3,2,1); plot(timeqr,Xg,'k',timeqr,Xqr,'g');title('Confronto X QR');
legend('Gazebo','QR','Location','northwest'); xlabel('time [s]'); ylabel('X [m]');grid on;
subplot(3,2,2);plot(timeqr,Eaqr(:,1),'k');title('Errore assoluto'); xlabel('time [s]'); ylabel('abs. err. [m]');grid on;
subplot(3,2,3);plot(timeukf,Xg1,'k',timeukf,Xukf,'b');title('Confronto X UKF');
legend('Gazebo','UKF','Location','northwest'); xlabel('time [s]'); ylabel('X [m]');grid on;
subplot(3,2,4);plot(timeukf,Eaukf(:,1),'k');title('Errore assoluto');xlabel('time [s]'); ylabel('abs. err. [m]');grid on;
subplot(3,2,5);plot(timeukfqr,Xg2,'k',timeukfqr,Xukfqr,'r');title('Confronto X UKFQR');
legend('Gazebo','UKFQR','Location','northwest'); xlabel('time [s]'); ylabel('X [m]');grid on;
subplot(3,2,6);plot(timeukfqr,Eaukfqr(:,1),'k'); title('Errore assoluto');xlabel('time [s]'); ylabel('abs. err. [m]');grid on;

figure
subplot(3,2,1); plot(timeqr,Yg,'k'); hold on; plot(timeqr,Yqr,'g');title('Confronto Y QR');
legend('Gazebo','QR','Location','northwest'); xlabel('time [s]'); ylabel('Y [m]');grid on;
subplot(3,2,2); plot(timeqr,Eaqr(:,2),'k'); title('Errore assoluto'); xlabel('time [s]'); ylabel('abs. err. [m]');grid on;
subplot(3,2,3); plot(timeukf,Yg1,'k'); hold on; plot(timeukf,Yukf,'b');title('Confronto Y UKF');
legend('Gazebo','UKF','Location','northwest'); xlabel('time [s]'); ylabel('Y [m]');grid on;
subplot(3,2,4); plot(timeukf,Eaukf(:,2),'k'); title('Errore assoluto');xlabel('time [s]'); ylabel('abs. err. [m]');grid on;
subplot(3,2,5); plot(timeukfqr,Yg2,'k'); hold on; plot(timeukfqr,Yukfqr,'r'); title('Confronto Y UKFQR');
legend('Gazebo','UKFQR','Location','northwest'); xlabel('time [s]'); ylabel('Y [m]');grid on;
subplot(3,2,6); plot(timeukfqr,Eaukfqr(:,2),'k'); title('Errore assoluto');xlabel('time [s]'); ylabel('abs. err. [m]');grid on;

figure
dist = [MSE(1) MSE(2) MSE(3)];
bar(dist)
ylabel('Errore Quadratico Medio [m]');
X=[1,2,3];
labels = arrayfun(@(value) num2str(value,'%2.2f'),dist,'UniformOutput',false);
text(X,dist,labels,'HorizontalAlignment','center','VerticalAlignment','bottom') 
set(gca,'xticklabel',{'MSE QR','MSE UKF','MSEUKFQR'});

% %Integral Absolute Error
% IAEX=[sum(Eaukf(:,1)); sum(Eaukfqr(:,1))]
% IAEY=[sum(Eaukf(:,2)); sum(Eaukfqr(:,2))]
% %Modulo Errore Finale
% distUKF = sqrt(Xukf.^2 + Yukf.^2); distQRukf = sqrt(Xabs1.^2 + Yabs1.^2);
% distUKFQR = sqrt(Xukfqr.^2 + Yukfqr.^2); distQRukfqr = sqrt(Xabs2.^2 + Yabs2.^2);
% MEF=[abs(distUKF(length(distUKF))-distQRukf(length(distQRukf)));...
%      abs(distUKFQR(length(distUKFQR))-distQRukfqr(length(distQRukfqr)))]
% figure
% x = categorical({'QR','UKF','UKFQR'});
% IAE = [IAEX IAEY];
% b=bar(x,IAE,'EdgeColor', 'black'); title('Integral Absolute Error X and Y'); ylabel('[m]');
% b(1).FaceColor='cyan';
% b(2).FaceColor='yellow';