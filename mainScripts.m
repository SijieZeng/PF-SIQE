N = 100;
k = 10; % timestep
constantSpeed = 10;
me = 0; % mean of gauss function    
sigma = 1; % standard deviation of gauss function
a = 0;
b = 1;
[x, z, i, sysNoise, measureNoise] = generate(N, k, constantSpeed); % generating x z i
pG = gauss(z, x, me, sigma); %  univariate
pU = pUniform(x, a, b, k, constantSpeed);

%% store parameters
%parameters = table;
%parameters.i = i';
%parameters.x = x';
%parameters.z = z';
%parameters.sysNoise = sysNoise';
%parameters.measureNoise = measureNoise';
%parameters.pG = pG';
%parameters.pU = pU';
%% Store parameters in a table with cell arrays for matrices
parameters = table;
parameters.i = {i}; % i as a row vector, stored in a cell
parameters.x = {x'}; % x matrix transposed to fit the table, stored in a cell
parameters.z = {z'}; % z matrix transposed to fit the table, stored in a cell
parameters.sysNoise = {sysNoise'}; % sysNoise matrix transposed, stored in a cell
parameters.measureNoise = {measureNoise'}; % measureNoise matrix transposed, stored in a cell
parameters.pG = {pG'}; % pG matrix transposed to fit the table, stored in a cell
parameters.pU = {pU'}; % pU matrix transposed to fit the table, stored in a cell
% access
% 提取z矩阵:
%zMatrix = parameters.z{1}; % 因为z存储在单元格中，使用{1}访问单元格内容
% 访问第10个粒子在时间步2的z值: zmatrix(i,idx)
%zValue = zMatrix(2, 10);
%如果您有 parameters.z{2}，这可能是因为：
%存储了不同的数据集：例如，如果您在相同的表格中存储了来自不同仿真场景的测量值，
% parameters.z{1} 可能包含第一组测量值，而 parameters.z{2} 包含第二组测量值。
%% 

