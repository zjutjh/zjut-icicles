function [gsrv1,gsrv2]=gngauss(m,sgma)
% [gsrv1,gsrv2]=gngauss(m,sgma)
% [gsrv1,gsrv2]=gngauss(sgma)
% [gsrv1,gsrv2]=gngauss
%		GNGAUSS  generates two independent Gaussian random variables with mean
%   		m and standard deviation sgma. If one of the input arguments is missing 
%   		it takes the mean as 0, and the standard deviation as the given parameter.
%   		If neither mean nor the variance is given, it generates two standard
%   		Gaussian random variables. 
if nargin == 0,
  m=0; sgma=1;
elseif nargin == 1,
  sgma=m; m=0;
end;
u=rand;                         	% a uniform random variable in (0,1)       
z=sgma*(sqrt(2*log(1/(1-u))));  	% a Rayleigh distributed random variable
u=rand;                         	% another uniform random variable in (0,1)
gsrv1=m+z*cos(2*pi*u);
gsrv2=m+z*sin(2*pi*u);