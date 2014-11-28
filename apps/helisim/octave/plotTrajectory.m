addpath('octave')
data = loadData('simlog.mat');
figure(1);
close;
M = 2; N = 3;
for Ni = 1:N
    subplot(M,N,Ni);
    title('X');
    plot(data.time, data.X(Ni,:)); hold on;
    plot(data.time, data.sX(Ni,:), 'r');
    axis([1, 2000, -50, 50])
    subplot(M,N,N + Ni);
    title('V');
    plot(data.time, data.V(Ni,:)); hold on;
    plot(data.time, data.sV(Ni,:), 'r');
    axis([1, 2000, -50, 50])
end
