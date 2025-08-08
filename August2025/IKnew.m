% Inverse Kinematics (IK)
function [Q, valid] = IKnew(x,y,z)
    % Joint limits
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07; q2_max = 1.97;
    q3_min = -0.4; q3_max = 2.92;
    
    % Link lengths
    l1=0.208; 
    l2=0.168;
    
    % Calculate inverse kinematics
    q1=atan2(x,z+l1);
    
    % Check if q1 is within limits
    if q1 < q1_min || q1 > q1_max
        valid = false;
        Q = [q1, 0, 0];
        warning('Joint 1 out of limits: [%.4f, %.4f]', q1_min, q1_max);
        return;
    end

    R=sqrt(x^2+(z+l1)^2);
    r=sqrt(x^2+(y-l2)^2+(z+l1)^2);
    
    Beta=atan2(y-l2,R);
    Gamma=acos((l1^2+r^2-l2^2)/(2*l1*r));
    
    q2=Gamma+Beta;

    Alpha=acos((l1^2+l2^2-r^2)/(2*l1*l2));
    
    q3=q2+Alpha-pi/2;

    Q=[q1,q2,q3];
    
    % Check basic joint limits
    valid = true;
    if q2 < q2_min || q2 > q2_max
        valid = false;
        warning('Joint 2 out of limits: [%.4f, %.4f]', q2_min, q2_max);
    end
    
    if q3 < q3_min || q3 > q3_max
        valid = false;
        warning('Joint 3 out of limits: [%.4f, %.4f]', q3_min, q3_max);
    end
    
    % Check relationship between joint 2 and joint 3
    if valid
        % When Joint 2 is at position x, Joint 3 range is [x-0.86, x+0.95]
        q3_min_rel = q2 - 0.86;
        q3_max_rel = q2 + 0.95;
        
        % When Joint 3 is at position -x, Joint 2 range is [x-0.95, x+0.93]
        % This is equivalent to: when Joint 2 is at position x, Joint 3 range is [x-0.93, x+0.95]
        % We'll use the more restrictive constraint
        q3_min_rel2 = q2 - 0.93;
        q3_max_rel2 = q2 + 0.95;
        
        % Take the more restrictive limits
        q3_min_rel = max(q3_min_rel, q3_min_rel2);
        q3_max_rel = min(q3_max_rel, q3_max_rel2);
        
        % Apply absolute limits
        q3_min_rel = max(q3_min_rel, q3_min);
        q3_max_rel = min(q3_max_rel, q3_max);
        
        if q3 < q3_min_rel || q3 > q3_max_rel
            valid = false;
            warning('Joint 3 out of relationship limits: [%.4f, %.4f] for q2=%.4f', q3_min_rel, q3_max_rel, q2);
        end
    end
end
