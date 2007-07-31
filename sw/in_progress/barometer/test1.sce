clear();
getf('baro_utils.sci');

filename = "data/07_07_26__16_37_09.baro.txt";
[time, gps_alt, pressure, gps_climb, temp] = baro_read_log(filename);

plot2d(time, gps_alt);