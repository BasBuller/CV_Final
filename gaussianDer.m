function Gd = gaussianDer(sigma)
x = -3.*sigma:3.*sigma;
Gd = -x./(sigma.^2).*gaussian(sigma);
end