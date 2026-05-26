% 计算不同腿长下适合的K矩阵，再进行多项式拟合，得到2*4矩阵每个参数对应的多项式参数
tic
j = 1;
leg = 0.12:0.02:0.28;

for i = leg
    k = LQR_k_calc(i);
    k11(j) = k(1,1);
    k12(j) = k(1,2);
    k13(j) = k(1,3);
    k14(j) = k(1,4);
    
    k21(j) = k(2,1);
    k22(j) = k(2,2);
    k23(j) = k(2,3);
    k24(j) = k(2,4);
    j = j+1;
end

% 多项式拟合（3次多项式，返回4个系数）
a11 = polyfit(leg, k11, 3);
a12 = polyfit(leg, k12, 3);
a13 = polyfit(leg, k13, 3);
a14 = polyfit(leg, k14, 3);

a21 = polyfit(leg, k21, 3);
a22 = polyfit(leg, k22, 3);
a23 = polyfit(leg, k23, 3);
a24 = polyfit(leg, k24, 3);

% 将多项式系数矩阵组合
c = [a11; a12; a13; a14; a21; a22; a23; a24];

% 初始化 combinedMatrix 为一个 cell 数组
combinedMatrix = cell(size(c, 1), 1);

% 循环遍历 c 的每一行
for x = 1:8
    % 获取当前行的元素，并将它们转换为字符串，用逗号分隔
    elements = arrayfun(@num2str, c(x, :), 'UniformOutput', false);
    str_row = strjoin(elements, ', ');
    
    % 将字符串包裹在花括号内
    str_row = ['{' str_row '}'];
    
    % 将字符串存储在 combinedMatrix 的相应单元格中
    combinedMatrix{x} = str_row;
end

% 使用 fprintf 输出
for i = 1:length(combinedMatrix)
    fprintf('%s\n', combinedMatrix{i});
end

% 多项式求值和绘图
x0 = leg;
y11 = polyval(a11, x0);
y12 = polyval(a12, x0);
y13 = polyval(a13, x0);
y14 = polyval(a14, x0);

y21 = polyval(a21, x0);
y22 = polyval(a22, x0);
y23 = polyval(a23, x0);
y24 = polyval(a24, x0);

% 创建2x4的子图布局（因为有8个参数）
figure;

subplot(2,4,1); plot(leg, k11, 'o', x0, y11, 'r'); 
xlabel('leg length'); ylabel('value'); title('k11');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,2); plot(leg, k12, 'o', x0, y12, 'r');
xlabel('leg length'); ylabel('value'); title('k12');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,3); plot(leg, k13, 'o', x0, y13, 'r');
xlabel('leg length'); ylabel('value'); title('k13');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,4); plot(leg, k14, 'o', x0, y14, 'r');
xlabel('leg length'); ylabel('value'); title('k14');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,5); plot(leg, k21, 'o', x0, y21, 'r');
xlabel('leg length'); ylabel('value'); title('k21');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,6); plot(leg, k22, 'o', x0, y22, 'r');
xlabel('leg length'); ylabel('value'); title('k22');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,7); plot(leg, k23, 'o', x0, y23, 'r');
xlabel('leg length'); ylabel('value'); title('k23');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

subplot(2,4,8); plot(leg, k24, 'o', x0, y24, 'r');
xlabel('leg length'); ylabel('value'); title('k24');
grid on; set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);

% 输出多项式系数
fprintf('fp32 a11[4] = {%.4f, %.4f, %.4f, %.4f};\n', a11(1), a11(2), a11(3), a11(4));
fprintf('fp32 a12[4] = {%.4f, %.4f, %.4f, %.4f};\n', a12(1), a12(2), a12(3), a12(4));
fprintf('fp32 a13[4] = {%.4f, %.4f, %.4f, %.4f};\n', a13(1), a13(2), a13(3), a13(4));
fprintf('fp32 a14[4] = {%.4f, %.4f, %.4f, %.4f};\n', a14(1), a14(2), a14(3), a14(4));

fprintf('fp32 a21[4] = {%.4f, %.4f, %.4f, %.4f};\n', a21(1), a21(2), a21(3), a21(4));
fprintf('fp32 a22[4] = {%.4f, %.4f, %.4f, %.4f};\n', a22(1), a22(2), a22(3), a22(4));
fprintf('fp32 a23[4] = {%.4f, %.4f, %.4f, %.4f};\n', a23(1), a23(2), a23(3), a23(4));
fprintf('fp32 a24[4] = {%.4f, %.4f, %.4f, %.4f};\n', a24(1), a24(2), a24(3), a24(4));

toc