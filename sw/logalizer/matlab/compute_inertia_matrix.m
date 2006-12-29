%
%compute interia matrix of approximate quadrotor
%

% l is the length of one arm ( 0.25 m )
%         coord of CG    mass        comment
% motorN   ( l  0 0)      90         motor + prop + prop adapter
% motorE   ( 0  l 0)      90         motor + prop + prop adapter
% motorS   (-l  0 0)      90         motor + prop + prop adapter
% motorW   ( 0 -l 0)      90         motor + prop + prop adapter
%
%  
%
%
%
%
%
clear

l=0.25;
quad(1) = struct('name', 'motorN', 'pos', [ l  0 0]', 'mass', 0.125);
quad(2) = struct('name', 'motorE', 'pos', [ 0  l 0]', 'mass', 0.125);
quad(3) = struct('name', 'motorS', 'pos', [-l  0 0]', 'mass', 0.125);
quad(4) = struct('name', 'motorW', 'pos', [ 0 -l 0]', 'mass', 0.125);

total_mass = 0.;
inertia = zeros(3,3);
for idx=1:length(quad)
  total_mass = total_mass + quad(idx).mass;
  m = quad(idx).mass;
  p = quad(idx).pos;
  inertia(1,1) = inertia(1,1) + m * (p(2)^2 + p(3)^2);
  inertia(2,2) = inertia(2,2) + m * (p(1)^2 + p(3)^2);
  inertia(3,3) = inertia(3,3) + m * (p(1)^2 + p(2)^2);
  inertia(1,2) = inertia(1,2) - m * p(1) * p(2);
  inertia(1,3) = inertia(1,3) - m * p(1) * p(3);
  inertia(2,3) = inertia(2,3) - m * p(2) * p(3);
end;

inertia(2,1) = inertia(1,2);
inertia(3,1) = inertia(1,3);
inertia(3,2) = inertia(2,3);

disp(sprintf('total mass %.0f g', total_mass*1000));

inertia