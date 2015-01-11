addpath('octave')
data = loadData('simlog.mat');
close all;
M = 2; N = 3;
figure(1);
for Ni = 1:N
    subplot(M,N,Ni); hold on;
    plot(data.time, data.X(Ni,:), 'c--p');
    plot(data.time, data.sX(Ni,:), 'k+');
    %axis([1, 2000, -50, 50]);
    subplot(M,N,N + Ni); hold on;
    plot(data.time, data.V(Ni,:), 'c--p');
    plot(data.time, data.sV(Ni,:), 'k+');
    plot(data.time, data.rV(Ni,:), 'ro');
    %axis([1, 2000, -50, 50]);
end
M = 2; N = 4;
figure(2);
for Ni = 1:N
    subplot(M,N,Ni); hold on;
    plot(data.time, data.Q(Ni,:), 'c--p');
    plot(data.time, data.sQ(Ni,:), 'k+');
    if Ni < 4
        plot(data.time, data.rQ(Ni,:), 'ro');
        subplot(M,N,N + Ni); hold on;
        plot(data.time, data.W(Ni,:), 'c--p');
        plot(data.time, data.sW(Ni,:), 'k+');
        plot(data.time, data.rW(Ni,:), 'ro');
        %axis([1, 2000, -50, 50]);
    end
end
%figure(1);
%subplot(M,N,1); title('X');
%subplot(M,N,2); title('Y');
%subplot(M,N,3); title('Z');

%subplot(M,N,1 + 0*1); ylabel('X');
%subplot(M,N,1 + 2*1); ylabel('V');
%subplot(M,N,1 + 1*1); ylabel('Q');
%subplot(M,N,1 + 3*1); ylabel('W');

figure(3);
subplot(2,2,1); plot(data.time, data.uth(1,:));
subplot(2,2,2); plot(data.time, data.uth(2,:));
subplot(2,2,3); plot(data.time, data.uth(3,:));
subplot(2,2,4); plot(data.time, data.utht(1,:));


