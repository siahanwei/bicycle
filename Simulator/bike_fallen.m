function [value, isterminal, direction] = bike_fallen(t,x)
value      = (abs(x(3)) - 90);
isterminal = 1;   % Stop the integration
direction  = 0;

end