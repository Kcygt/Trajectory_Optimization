function Q = IK(x,y,z)

    %% LINK LENGTHS
    l1 = 0.208;
    l2 = 0.168;

    %% BASE ROTATION

    q1 = atan2(x,z+l1);

    %% DISTANCES

    R = sqrt(x.^2 + (z+l1).^2);

    r = sqrt(x.^2 + (y-l2).^2 + (z+l1).^2);

    %% WORKSPACE CHECK

    if r > (l1 + l2)

        error('Target point outside workspace');

    end

    %% BETA

    Beta = atan2(y-l2,R);

    %% GAMMA

    val1 = (l1^2 + r.^2 - l2^2) ./ (2*l1*r);

    val1 = max(min(val1,1),-1);

    Gamma = acos(val1);

    %% q2

    q2 = Gamma + Beta;

    %% ALPHA

    val2 = (l1^2 + l2^2 - r.^2) ./ (2*l1*l2);

    val2 = max(min(val2,1),-1);

    Alpha = acos(val2);

    %% q3

    q3 = q2 + Alpha - pi/2;

    %% OUTPUT

    Q = [q1,q2,q3];

end