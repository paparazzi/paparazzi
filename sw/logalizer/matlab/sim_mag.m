function [mag] = sim_mag(h_ned, dcm)

mag_noise = 0.0;
h_bod = dcm * h_ned;

mag = h_bod + mag_noise * ( 1 - 2 * rand(3,1));