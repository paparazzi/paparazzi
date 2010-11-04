%% unwrap
%
% [unwraped] = unwrap(wraped)
%
% 
function [unwraped] = unwrap(wraped)

   unwraped = zeros(length(wraped), 1);
   cnt = 0;
   for i=2:length(wraped)
       dif = wraped(i) - wraped(i-1);
       if (dif > pi/2)
       	cnt=cnt-1;
       elseif (dif <-pi/2)
       	cnt=cnt+1;
       end
       unwraped(i) = wraped(i)+2*pi*cnt;
    end
endfunction