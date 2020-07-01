clear all
clc
format long
n='';
file=strcat('jointData',n,'.txt');
fid = fopen(file);
[p0 p1]=textread(file, '%f %f');
fclose(fid);

t=1:1:length(p0);
figure
plot(t,p0*0.2,'b');
hold on
plot(t,p1*0.2,'r');