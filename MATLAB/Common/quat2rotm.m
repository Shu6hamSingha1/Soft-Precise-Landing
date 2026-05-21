function R = quat2rotm(q)
%QUAT2ROTM Convert quaternion(s) [w x y z] to 3x3 rotation matrices.
%   Standalone shim — replaces the Robotics System Toolbox version so the
%   project can run on base MATLAB.  Input q is N×4 (rows are quats); output
%   is 3×3×N (3×3 for a single quat).  Quats must be unit norm.

    if size(q, 2) ~= 4
        error('quat2rotm:badShape', 'quaternions must be N×4 ([w x y z])');
    end
    N = size(q, 1);
    R = zeros(3, 3, N);
    for k = 1:N
        w = q(k,1); x = q(k,2); y = q(k,3); z = q(k,4);
        % normalize (defensive)
        n = sqrt(w*w + x*x + y*y + z*z);
        if n > 0
            w = w/n; x = x/n; y = y/n; z = z/n;
        end
        R(:,:,k) = [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w);
                    2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w);
                    2*(x*z - y*w),      2*(y*z + x*w),      1 - 2*(x*x + y*y)];
    end
    if N == 1
        R = R(:,:,1);
    end
end
