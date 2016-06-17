clear();
clf();

//##########
//##########
function [rm_x, rm_y, rm_z, ra_x, ra_y, ra_z] = read_log(filename)
  
  rm_x=[];
  rm_y=[];
  rm_z=[];
  
  ra_x=[];
  ra_y=[];
  ra_z=[];
  
  u=mopen(filename,'r');
  
  while meof(u) == 0,
    line = mgetl(u, 1);
    if strindex(line, '#') ~= 1 & length(line) ~= 0,
      [nb_scan, _mz, _my, _mx] = msscanf(1, line, '148 IMU_MAG_RAW %f %f %f');
      if nb_scan == 3,
        rm_x = [rm_x _mx];
        rm_y = [rm_y _my];
	rm_z = [rm_z _mz];
      else
	[nb_scan, _ax, _ay, _az] = msscanf(1, line, '148 IMU_ACCEL_RAW %f %f %f');	
	if nb_scan == 3,
	  ra_x = [ra_x _ax];
	  ra_y = [ra_y _ay];
	  ra_z = [ra_z _az];
	end
      end
    end
  end
  
  mclose(u);
  
endfunction

//##########
//##########
function [nx, ny, nz, gx, gy, gz] = min_max_calib(dx, dy, dz)
  min_x = min(dx);
  max_x = max(dx);
  nx = (max_x + min_x) / 2.;
  gx = (max_x - min_x) / 2.;
  
  min_y = min(dy);
  max_y = max(dy);
  ny = (max_y + min_y) / 2.;
  gy = (max_y - min_y) / 2.;

  min_z = min(dz);
  max_z = max(dz);
  nz = (max_z + min_z) / 2.;
  gz = (max_z - min_z) / 2.;
endfunction

[rm_x, rm_y, rm_z, ra_x, ra_y, ra_z] = read_log('log_magnetometer');

[m_mm_nx, m_mm_ny, m_mm_nz, m_mm_gx, m_mm_gy, m_mm_gz] = ...
    min_max_calib(rm_x, rm_y, rm_z);

[a_mm_nx, a_mm_ny, a_mm_nz, a_mm_gx, a_mm_gy, a_mm_gz] = ...
    min_max_calib(ra_x, ra_y, ra_z);

m_mm_x = (rm_x - m_mm_nx) / m_mm_gx;
m_mm_y = (rm_y - m_mm_ny) / m_mm_gy;
m_mm_z = (rm_z - m_mm_nz) / m_mm_gz;

printf('mx : n -> %f  g -> %f\n', m_mm_nx, m_mm_gx);
printf('my : n -> %f  g -> %f\n', m_mm_ny, m_mm_gy);
printf('mz : n -> %f  g -> %f\n', m_mm_nz, m_mm_gz);

m_mm_norm = sqrt(m_mm_x^2 + m_mm_y^2 + m_mm_z^2);
idx_m = 1:length(rm_z); 


//a_mm_x = (ra_x - a_mm_nx) / a_mm_gx;
//a_mm_y = (ra_y - a_mm_ny) / a_mm_gy;
//a_mm_z = (ra_z - a_mm_nz) / a_mm_gz;

//a_mm_norm = sqrt(a_mm_x^2 + a_mm_y^2 + a_mm_z^2);
//idx_a = 1:length(ra_z); 

subplot(3,2,1);
param3d(rm_x, rm_y, rm_z);
subplot(3,2,2);
plot(idx_m, rm_x);

//subplot(3,2,2);
//param3d(ra_x, ra_y, ra_z);
subplot(3,2,3);
plot(idx_m, m_mm_norm, idx_m, ones(1, length(rm_z)));
//subplot(3,2,4);
//plot(idx_a, a_mm_norm, idx_a, ones(1, length(ra_z)));

subplot(3,2,5);
alpha = 0:0.1:2*%pi;
c_x = m_mm_nx+ m_mm_gx * cos(alpha);
c_y = m_mm_ny+ m_mm_gy * sin(alpha);
plot(rm_x, rm_y, c_x, c_y)

//subplot(3,2,6);
//c_x = a_mm_nx+ a_mm_gx * cos(alpha);
//c_y = a_mm_ny+ a_mm_gy * sin(alpha);
//plot(ra_x, ra_y, c_x, c_y)


//
//
//

function err = cost_fun(p, z)
  err = (z(1) - p(1))^2 / p(4)^2 + ...
	(z(2) - p(2))^2 / p(5)^2 + ...
	(z(3) - p(3))^2 / p(6)^2 - 1;
endfunction

if 0
  p0 = [ mx_neutral; my_neutral; mz_neutral; mx_gain; my_gain; mz_gain];
  Z = [rm_x; rm_y; rm_z];
  [p, err] = datafit(cost_fun, Z, p0)
else
  // expe 0
  p = [  1925.0482  
	 2124.7774  
	 1975.155   
	 446.40699  
	 426.51219  
	 412.88952 ];
  // expe 1 cut
  p  = [  1933.2636  
	  2136.651   
	  1977.4088  
	  464.52769  
	  452.71114  
	  433.81507 ];
  // expe 2 uncut
  p  = [ 1931.7855  
	 2114.7077  
	 1976.6819  
	 444.15179  
	 472.04251  
	 479.76116 ];
  // expe 2 cut
  p  = [ 1931.1511  
	 2145.2051  
	 1977.5055  
	 467.83817  
	 450.52012  
	 447.26642  ];

end

opt_nx = p(1);
opt_ny = p(2);
opt_nz = p(3);
opt_gx = p(4);
opt_gy = p(5);
opt_gz = p(6);
opt_mx = (rm_x - opt_nx) / opt_gx;
opt_my = (rm_y - opt_ny) / opt_gy;
opt_mz = (rm_z - opt_nz) / opt_gz;

//opt_norm = sqrt(opt_mx^2 + opt_my^2 + opt_mz^2);
//subplot(2,2,3);
//plot(idx, opt_norm, 'cya+');
