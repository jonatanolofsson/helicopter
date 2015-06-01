close all;
motioncontrol = load('motioncontrol.mat');
N = size(motioncontrol, 1);
time = 1:N;
F = reshape(motioncontrol(:,1:36), 6, 6, []);
B = reshape(motioncontrol(:,37:end), 4, 4, []);
eigsF = zeros(6, N);
controllable = zeros(1, N);
for t = time
    eigsF(:,t) = eig(F(:,:,t));
    %ctrb_mat = ctrb(F(:,:,t));
    %controllable(t) = rank(ctrb_mat);
end
figure();
plot(time, eigsF)
