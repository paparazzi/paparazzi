clear();
clearglobal();


exec('q3d_utils.sci');
exec('q3d_fdm.sci');
exec('q3d_ctl.sci');
exec('q3d_ref_misc.sci');

a = [ 0; 0 
      0; 0
      0; 0 
      0; 0
      0; 0 ];
b = [ 0        ;  0
      1.2566371;  0
      0        ;  1.5791367 
     -1.9844017;  0
      0        ; -2.4936727 ];
[time, Xref] = get_reference_poly(1, a, b);

clf();

subplot(5,2,1);
plot2d(time, Xref(1,:));
plot2d(0, a(1), -3);
plot2d(1, b(1), -3);
xtitle('X(0)');

subplot(5,2,2);
plot2d(time, Xref(2,:));
plot2d(0, a(2), -3);
plot2d(1, b(2), -3);
xtitle('Z(0)');

subplot(5,2,3);
plot2d(time, Xref(3,:));
plot2d(0, a(3), -3);
plot2d(1, b(3), -3);
xtitle('X(1)');

subplot(5,2,4);
plot2d(time, Xref(4,:));
plot2d(0, a(4), -3);
plot2d(1, b(4), -3);
xtitle('Z(1)');

subplot(5,2,5);
plot2d(time, Xref(5,:));
plot2d(0, a(5), -3);
plot2d(1, b(5), -3);
xtitle('X(2)');

subplot(5,2,6);
plot2d(time, Xref(6,:));
plot2d(0, a(6), -3);
plot2d(1, b(6), -3);
xtitle('Z(2)');

subplot(5,2,7);
plot2d(time, Xref(7,:));
plot2d(0, a(7), -3);
plot2d(1, b(7), -3);
xtitle('X(3)');

subplot(5,2,8);
plot2d(time, Xref(8,:));
plot2d(0, a(8), -3);
plot2d(1, b(8), -3);
xtitle('Z(3)');

subplot(5,2,9);
plot2d(time, Xref(9,:));
plot2d(0, a(9), -3);
plot2d(1, b(9), -3);
xtitle('X(4)');

subplot(5,2,10);
plot2d(time, Xref(10,:));
plot2d(0, a(10), -3);
plot2d(1, b(10), -3);
xtitle('Z(4)');
