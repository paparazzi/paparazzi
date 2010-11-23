
function povray_draw( time, diff_flat_ref )

  dt_display = 1/25;

  t_idx = 1;
  f_idx = 1;
  t = time(t_idx);
  while (t_idx<length(time))
    printf('drawing %d (%f)\n', f_idx, time(t_idx));
    x = -1000*diff_flat_ref(FDM_SX,t_idx);
    y = 1000*diff_flat_ref(FDM_SZ,t_idx);
    alpha =  deg_of_rad(-diff_flat_ref(FDM_STHETA,t_idx));
//    printf('( %f %f %f)\n', x, y, alpha);
    fid = mopen('povray/q3d.pov', "w");
    mfprintf(fid, "#include ""povray/q3d.inc""\n");
    mfprintf(fid, "object { Q3D()  rotate z*%f translate <%f,%f,0>}",alpha,x,y);
    mclose(fid);
    cmd = sprintf('povray povray/q3d.pov +Opovray/foo%04d.png Display=false +W800 +H600 +Q9 +A0.3 +R5', f_idx);
    a = unix_g(cmd);
    while (t_idx<length(time) & time(t_idx) < f_idx*dt_display)
      t_idx = t_idx + 1;
    end
    f_idx = f_idx + 1;
  end

  mplayer_cmd = "mencoder ""mf://povray/foo*.png"" -mf fps=25 -o povray/test.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=800";
  unix_g(mplayer_cmd);

endfunction

