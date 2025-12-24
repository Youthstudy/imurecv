T = readtable('joint_5.csv');

% 分类的逻辑索引
idx1 = T{:,2} == 1;
idx2 = T{:,2} == 2;

% 第6列：y
y1 = T{idx1, 6};
y2 = T{idx2, 6};

% 第3列：posrel
posrel1 = T{idx1, 3};
posrel2 = T{idx2, 3};

% 第5列：t
t1 = T{idx1, 5};
t2 = T{idx2, 5};

p1 = T{idx1,"mode"};
p2 = T{idx2,"mode"};

%%
fre = 0.001;
% scatter(posrel1(:),t1(:));
% subplot(2,1,1)
x1 = (0:fre:3)';
% plot(x1,y1(1:size(x1,1)));hold on;
% % plot(x1,f1(x1),'LineWidth',2);
% plot(x1,posrel1(1:size(x1,1)), 'LineWidth',1.5);
% 
% legend('pos_{cmd}','pos_d','pos_{rel}')
% subplot(2,1,2)
% x2 = (0:fre:3)';
% plot(x2,y2(1:size(x2,1)));hold on;
% % plot(x2,-f2(x2),'LineWidth',2);
% plot(x2,posrel2(1:size(x2,1)), 'LineWidth',1.5);
% legend('pos_{cmd}','pos_d','pos_{rel}')

