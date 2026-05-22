function R = roty(deg)
%ROTY Rotation matrix about y-axis, angle in DEGREES.  Standalone shim
%   (avoids Phased Array System Toolbox dependency).
    rad = deg * pi / 180;
    c = cos(rad); s = sin(rad);
    R = [ c, 0, s;
          0, 1, 0;
         -s, 0, c];
end
