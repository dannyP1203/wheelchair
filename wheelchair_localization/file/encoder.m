clear all
clc
format long
n='';
file=strcat('odomstimaData',n,'.txt');
fid = fopen(file);
[deltaD, deltaS, distD, distS, deltaC, dist, tt, x, y, thetaZ]=textread(file, '%f %f %f %f %f %f %f %f %f %f');
fclose(fid);

t=1:1:length(deltaD);
figure
subplot(2,3,1); plot(t,deltaD); title('deltaD');
subplot(2,3,2); plot(t,deltaS); title('deltaS');
subplot(2,3,3); plot(t,deltaC); title('deltaC');
subplot(2,3,4); plot(t,distD);  title('Ruota DX');
subplot(2,3,5); plot(t,distS); title('Ruota SX');
subplot(2,3,6); plot(t,dist); title('Distanza');
figure
subplot(1,3,1); plot(t,x);title('X');
subplot(1,3,2); plot(t,y);title('Y');
subplot(1,3,3); plot(x,y); title('XY');