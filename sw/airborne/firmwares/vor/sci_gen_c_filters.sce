//
//
// Generates C file and corresponding headers for filters used in
// C implementation
//

clear();

exec('sci_vor_filters.sci');
[filters] = vor_get_filters();

filter_name = [ "BP_VAR"
                "BP_REF"
                "LP_DECIM"
	        "LP_VAR"
	        "LP_REF"
	        "LP_FM" ];

function print_poly(fid, name, p, header)
  c_p = coeff(p);
  l_p = length(c_p);
  l_ps = name+'_LEN';
  if (header)
    fprintf(fid, '#define %s %i\n', l_ps, l_p);
    fprintf(fid, 'extern FLOAT_T %s[];\n', name);   
  else
    fprintf(fid, 'FLOAT_T %s[%s]={\n', name, l_ps);  
    for i=l_p:-1:1
      fprintf(fid,'%10.40f,\n',c_p(i));
    end
    fprintf(fid,'};\n');
  end
endfunction
	    
//
// header
//

filename = 'vor_filter_params'; 
fid = mopen(filename+'.h', 'w');

fprintf(fid,'#ifndef VOR_LF_FILTER_PARAMS_H\n');
fprintf(fid,'#define VOR_LF_FILTER_PARAMS_H\n\n');
	    
for i=1:FILTER_NB
  f = filters(i);
  fn = filter_name(i);
  fprintf(fid,'/* %s filter */\n', fn);
  print_poly(fid, fn+"_NUM", f.tf.num, 1);
  print_poly(fid, fn+"_DEN", f.tf.den, 1);
  fprintf(fid,'\n');
end

fprintf(fid,'#endif /* VOR_LF_FILTER_PARAMS_H */\n');
mclose(fid);

//
// C
//
  
fid = mopen(filename+'.c', 'w');
  fprintf(fid,'#include ""%s.h""\n\n', filename);
for i=1:FILTER_NB
  f = filters(i);
  fn = filter_name(i);
  print_poly(fid, fn+"_NUM", f.tf.num, 0);
  print_poly(fid, fn+"_DEN", f.tf.den, 0);
  fprintf(fid,'\n');
end

mclose(fid);
  

