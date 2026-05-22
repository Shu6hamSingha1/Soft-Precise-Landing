function R = rotx(deg)
%ROTX Rotation matrix about x-axis, angle in DEGREES.  Standalone shim
%   (avoids Phased Array System Toolbox dependency).
    rad = deg * pi / 180;
    c = cos(rad); s = sin(rad);
    R = [1, 0,  0;
         0, c, -s;
         0, s,  c];
end
