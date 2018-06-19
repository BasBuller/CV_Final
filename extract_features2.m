function [x1 y1 a1 b1 c1 desc1 x2 y2 a2 b2 c2 desc2] = extract_features(name,mode)
%EXTRACT_FEATURES Use extract features using harris or hessian affine
if(mode)
    command = strcat(strcat("/home/rick/Documents/extract_features/extract_features.ln -haraff -i ",name),' -sift -thres 10');
    system(command)
    command = strcat(strcat("/home/rick/Documents/extract_features/extract_features.ln -hesaff -i ",name),' -sift -thres 10');
    system(command)
end

imtxt = strcat(strcat(name),'.haraff.sift');
[x1 y1 a1 b1 c1 desc1] = loadFeatures(imtxt);

imtxt = strcat(strcat(name),'.hesaff.sift');
[x2 y2 a2 b2 c2 desc2] = loadFeatures(imtxt);

end

