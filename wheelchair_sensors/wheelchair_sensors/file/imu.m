clear all
clc
format long
n='';
file=strcat('imuData',n,'.txt');
fid = fopen(file);
[qx, qy, qz, q0, avx, avy, avz, lax, lay, laz, b, a, h]=textread(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

t=1:1:length(qx);
% figure
% subplot(3,4,1);  plot(t,qx); title('Orient.X');
% subplot(3,4,2);  plot(t,qy); title('Orient.Y');
% subplot(3,4,3);  plot(t,qz); title('Orient.Z');
% subplot(3,4,4);  plot(t,q0); title('Orient.W');
% subplot(3,4,5);  plot(t,avx);title('Ang.Vel.X');
% subplot(3,4,6);  plot(t,avy);title('Ang.Vel.Y');
% subplot(3,4,7);  plot(t,avz);title('Ang.Vel.Z');
% subplot(3,4,9);  plot(t,lax);title('Lin.Acc.X');
% subplot(3,4,10); plot(t,lay);title('Lin.Acc.Y');
% subplot(3,4,11); plot(t,laz);title('Lin.Acc.Z');
figure
subplot(2,4,1);  plot(t,qx); title('Orient.X');
subplot(2,4,2);  plot(t,qy); title('Orient.Y');
subplot(2,4,3);  plot(t,qz); title('Orient.Z');
subplot(2,4,4);  plot(t,q0); title('Orient.W');
subplot(2,4,5);  plot(t,b);title('Bank');
subplot(2,4,6);  plot(t,a);title('Attitude');
subplot(2,4,7);  plot(t,h);title('Heading');