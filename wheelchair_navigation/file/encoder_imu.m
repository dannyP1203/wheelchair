clear all
clc
format long
n='';
file=strcat('IMUodomstimaData',n,'.txt');
fid = fopen(file);
[Xi, Yi, Xe, Ye, Gx, Gy]=textread(file, '%f %f %f %f %f %f');
fclose(fid);

plot(Gx,Gy,'k'); hold on; plot(Xi,Yi,'g'); plot(Xe,Ye,'b');title('XY');legend('Gaz','Imu','Enc');