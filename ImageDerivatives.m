function F = ImageDerivatives(img, sigma_x,sigma_y, type)
if type == 'xx'
    x = -3*sigma_x:3*sigma_x;
    Id = (-(sigma_x).^2+x.^2)./((sigma_x).^4).*gaussian(sigma_x);
    F = conv2(img,Id,'same');
elseif type == 'yy'
    x = -3*sigma_y:3*sigma_y;
    Id = ((-(sigma_y).^2+x.^2)./((sigma_y).^4).*gaussian(sigma_y))';
    F = conv2(img,Id,'same');
elseif type == 'x'
    Id = gaussianDer(sigma_x);
    F = conv2(img,Id,'same');
elseif type == 'y'
    Id = (gaussianDer(sigma_y))';
    F = conv2(img,Id,'same');
elseif type == 'xy'
    Idx = gaussianDer(sigma_x);
    Idy = gaussianDer(sigma_y);
    F = conv2(Idx,Idy,img,'same');
elseif type == 'yx'
    Idx = gaussianDer(sigma_x);
    Idy = gaussianDer(sigma_y);
    F = conv2(Idy,Idx,img,'same');
end