

function povray_draw( time, diff_flat_ref )

  dt_display = 1/25;

  t_idx = 1;
  f_idx = 1;
  t = time(t_idx);
  while (t_idx<length(time))
    printf('drawing %d (%f)\n', f_idx, time(t_idx));
    px =  1000*diff_flat_ref(DF_REF_X,t_idx);
    py =  1000*diff_flat_ref(DF_REF_Y,t_idx);
    pz =  1000*diff_flat_ref(DF_REF_Z,t_idx);
    _phi   = deg_of_rad(diff_flat_ref(DF_REF_PHI,t_idx));
    _theta = deg_of_rad(diff_flat_ref(DF_REF_THETA,t_idx));
    _psi   = deg_of_rad(diff_flat_ref(DF_REF_PSI,t_idx));
//    printf('( %f %f %f - %f %f %f)\n', px, py, pz, alpha, beta, gamma);
    fid = mopen('povray/q6d.pov', "w");
    mfprintf(fid, "#include ""povray/q6d.inc""\n");
    mfprintf(fid, "object { Q6D(%f,%f,%f,%f,%f,%f)}\n",px,py,pz,_phi,_theta,_psi);
    mfprintf(fid, "object {  AXIS_NED()}\n");
    mclose(fid);
    cmd = sprintf('povray povray/q6d.pov +Opovray/img%04d.png Display=false +W800 +H600 +Q9 +A0.3 +R5', f_idx);
    a = unix_g(cmd);
    while (t_idx<length(time) & time(t_idx) < f_idx*dt_display)
      t_idx = t_idx + 1;
    end
    f_idx = f_idx + 1;
  end

  mplayer_cmd = "mencoder ""mf://povray/img*.png"" -mf fps=25 -o povray/q6d.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=800";
  unix_g(mplayer_cmd);

endfunction

