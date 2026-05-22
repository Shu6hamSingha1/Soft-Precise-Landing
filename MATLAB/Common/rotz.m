function R = rotz(deg)
%ROTZ Rotation matrix about z-axis, angle in DEGREES (toolbox convention).
%   Standalone — replaces the Phased Array System Toolbox version so the
%   project doesn't require that separately-licensed toolbox.
    rad = deg * pi / 180;
    c = cos(rad); s = sin(rad);
    R = [c, -s, 0;
         s,  c, 0;
         0,  0, 1];
end
