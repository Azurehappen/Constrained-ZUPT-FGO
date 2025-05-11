function R = Rt2b(ang)
    % ang = [roll, pitch, yaw];
    % euler angles defined from n to b;
    % DCM: from n to b;

    roll = ang(1); %1
    pitch = ang(2); %2
    yaw = ang(3); %3

    C3 = [cos(yaw) sin(yaw) 0;
          -sin(yaw) cos(yaw) 0;
          0 0 1];
    C2 = [cos(pitch) 0 -sin(pitch);
          0 1 0;
          sin(pitch) 0 cos(pitch)];
    C1 = [1 0 0;
          0 cos(roll) sin(roll);
          0 -sin(roll) cos(roll)];
    R = C1 * C2 * C3;

end
