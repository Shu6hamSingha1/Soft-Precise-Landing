function eul = quat2eul(q, seq)
%QUAT2EUL Convert quaternion(s) to Euler angles.
%   Standalone shim — replaces the Robotics System Toolbox version.
%   Input q is N×4 [w x y z]; output is N×3 angles in radians.
%
%   Supported sequences (matching MATLAB Robotics Toolbox conventions):
%     'ZYX' (default): eul = [yaw, pitch, roll]   — rotations about z, y, x
%     'XYZ':            eul = [phi_x, phi_y, phi_z] — rotations about x, y, z

    if nargin < 2; seq = 'ZYX'; end
    if size(q, 2) ~= 4
        error('quat2eul:badShape', 'quaternions must be N×4 ([w x y z])');
    end
    seq = upper(seq);
    if ~ismember(seq, {'ZYX', 'XYZ'})
        error('quat2eul:badSeq', 'Sequence %s not implemented (only ZYX, XYZ)', seq);
    end

    N = size(q, 1);
    eul = zeros(N, 3);
    for k = 1:N
        w = q(k,1); x = q(k,2); y = q(k,3); z = q(k,4);
        n = sqrt(w*w + x*x + y*y + z*z);
        if n > 0; w=w/n; x=x/n; y=y/n; z=z/n; end

        % Rotation matrix from quaternion (passive, standard right-handed)
        R = [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w);
             2*(x*y + z*w),      1 - 2*(x*x + z*z),  2*(y*z - x*w);
             2*(x*z - y*w),      2*(y*z + x*w),      1 - 2*(x*x + y*y)];

        switch seq
        case 'ZYX'
            yaw   = atan2(R(2,1), R(1,1));
            sinp  = -R(3,1);
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi/2;
            else
                pitch = asin(sinp);
            end
            roll  = atan2(R(3,2), R(3,3));
            eul(k,:) = [yaw, pitch, roll];

        case 'XYZ'
            % R = Rx(phi_x) * Ry(phi_y) * Rz(phi_z)  intrinsic
            % R(1,3) = sin(phi_y); R(1,1) = cos(phi_y)*cos(phi_z); etc.
            siny = R(1,3);
            if abs(siny) >= 1
                phi_y = sign(siny) * pi/2;
                % Gimbal lock — keep z=0, all rotation in x
                phi_x = atan2(-R(2,3), R(2,2));
                phi_z = 0;
            else
                phi_y = asin(siny);
                phi_x = atan2(-R(2,3), R(3,3));
                phi_z = atan2(-R(1,2), R(1,1));
            end
            eul(k,:) = [phi_x, phi_y, phi_z];
        end
    end
end
