Array=csvread('new.csv');
col1 = Array(:, 3);
col2 = Array(:, 4);
plot(col1, col2)