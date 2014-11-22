addpath('octave')
data = loadData('simlog.mat');
figure(1);
subplot(2,1,1);
plot(data.time, data.X);
subplot(2,1,2);
plot(data.time, data.uV);
