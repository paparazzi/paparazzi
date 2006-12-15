function [accel] = sim_accel(g_ned, dcm)

accel_noise = 5.;
%accel_noise = 0.;

g_bod = dcm * g_ned;

accel = g_bod + accel_noise * ( 1 - 2 * rand(3,1));