function R = rotz(deg)
%ROTZ Rotation matrix about z-axis, angle in DEGREES (toolbox convention).
%   Standalone shim — replaces the Phased Array System Toolbox version.
    rad = deg * pi / 180;
    c = cos(rad); s = sin(rad);
    R = [c, -s, 0;
         s,  c, 0;
         0,  0, 1];
end
