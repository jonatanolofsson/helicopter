data = loadData('../build/simlog.mat');
figure(1);
subplot(2,1,1);
plot(data.time, data.states);
subplot(2,1,2);
plot(data.time, data.control);
