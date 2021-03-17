function presenter

Array=csvread('filename.csv');%reads in CSV file

col1 = Array(:, 1);%takes column 1
col2 = Array(:, 2);%takes column 2
plot(col1, col2) %plots in XY coord
