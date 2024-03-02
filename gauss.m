function pG = gauss(z, x, me, sigma) 
%% GAUSS Gussian distribution function
% For the measurement function z_k = x_k + n_k, n_k ~ N[0,1], the probability 
% fuunction is p(z_k^i | x_k^i) = \frac{1}{\sqrt{2\pi}} e^{-\frac{1}{2}(z_k^i 
% - x_k^i)^2}. Since my simple case is univariate, the state only have one dimension 
% which is position of the vehicle.
% ahgauss(x, me, sigma)
% returns the probability density function value at x
% for a Gaussian distribution with mean me and standard deviation sigma.
% z     matrix of locations measurement 
% x     matrix of locations true
% me    mean of the distribution
% sigma standard deviation of the distribution
% Check for NaN inputs
if(any(isnan(z)))
    error('Input ''z'' contains NaN.');
end
if(any(isnan(x)))
    error('Input ''x'' contains NaN.');
end
if(any(isnan(me)))
    error('Input ''me'' is NaN.');
end
if ~isscalar(me)
    error('mean me must be a scalar');
end
if ~isscalar(sigma) || sigma <= 0
    error('sigma must be a positive scalar');
end
%pre-allocation, initialization
%pG = zeros(1, length(x));
% Calculate the probability density function value at x
pG = exp(-0.5 * ((z - x - me) / sigma).^2) / (sigma * sqrt(2*pi));
end