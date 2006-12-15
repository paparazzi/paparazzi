function [gyro] = sim_gyro_2(rates)

gyro_biases = [ 0.01 0.02 0.03]';
%gyro_biases = [ 0 0 0]';
gyro_noise = 0.03;

gyro = rates + gyro_biases + gyro_noise * ( 1 - 2 * rand(3,1));
