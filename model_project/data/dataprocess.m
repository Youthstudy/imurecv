clear,clc;

T = readtable('all_features_with_gait_phase.csv');
n = size(T);
y = [];
for i = 1:n(1)
    if T{i,124} > 0 && T{i,124} < 1
        y = [y; [T{i,122}, T{i,123}]];
    end
end

fre = 0.005;
x = 0:fre:(size(y,1)-1)*fre;
x = x';

f1 = fit(x, y(:,1), 'fourier3');
f2 = fit(x, y(:,2), 'fourier3');
figure;
subplot(2,1,1)
plot(x, y(:,1), 'o'); hold on;
plot(f1, 'r-');
legend('raw data', 'fourier fit');
subplot(2,1,2)
plot(x, y(:,2), 'o'); hold on;
plot(f2, 'r-');
legend('raw data', 'fourier fit');

figure;
x_t = 0:fre:(10);
x_t = x_t';
y = f1(x_t);
plot(x_t,y);
