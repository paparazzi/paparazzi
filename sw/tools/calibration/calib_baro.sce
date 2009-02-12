clear();

ac_id = 150
test = 1;

M=fscanfMat(sprintf('%d_ADC_GENERIC_%d', ac_id, test));

time    = M(:,1);
id      = M(:,2);
off     = M(:,3);
adc     = M(:,4);

select test
  case 1
    t1 = find(time > 0   & time < 40);
    t2 = find(time > 70  & time < 120);
    t3 = find(time > 220 & time < 250);
  case 2
    t3 = find(time > 40  & time < 50);
    t2 = find(time > 140 & time < 160);
    t1 = find(time > 190 & time < 210);
  end

v1 = mean(adc(t1))
v2 = mean(adc(t2))
v3 = mean(adc(t3))

k1 = (v1-v3)/9.
k2 = (v1-v2)/4.5
k3 = (v2-v3)/4.5

k = (k1 + k2 + k3) / 3

clf();

drawlater();

xtitle('X', 'time (s)','');
plot2d(time, adc, 4);
plot2d(time(t1), adc(t1), 1);
plot2d(time(t2), adc(t2), 2);
plot2d(time(t3), adc(t3), 3);

drawnow();