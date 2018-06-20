function [Zdist Xdist Ydist] = Zdist(rotations)

load xyz;

rotateX = rotations(1);
rotateY = rotations(2);
rotateZ = rotations(3);
Rx = [1 0 0; 0 cos(rotateX) -sin(rotateX); 0 sin(rotateX) cos(rotateX)];
Ry = [cos(rotateY) 0 sin(rotateY); 0 1 0; -sin(rotateY) 0 cos(rotateY)];
Rz = [cos(rotateZ) -sin(rotateZ) 0; sin(rotateZ) cos(rotateZ) 0; 0 0 1];
newZ =Rz * Ry * Rx * [x; y; z];
Zdist = max(newZ(3,:)) - min(newZ(3,:));
Xdist = min(newZ(1,:)) - max(newZ(1,:));
Ydist = min(newZ(2,:)) - max(newZ(2,:));

save newZ;



end