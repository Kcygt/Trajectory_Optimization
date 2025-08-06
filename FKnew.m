function [x,y,z,valid]=FKnew(q1,q2,q3)
    % Joint limits
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07; q2_max = 1.97;
    q3_min = -0.4; q3_max = 2.92;
    
    % Link lengths
    l1=0.208; 
    l2=0.168;
    
    % Store original values for validation
    q1_orig = q1;
    q2_orig = q2;
    q3_orig = q3;
    
    % Clamp joint values to limits
    q1 = max(q1_min, min(q1_max, q1));
    q2 = max(q2_min, min(q2_max, q2));
    q3 = max(q3_min, min(q3_max, q3));
    
    % Check relationship between joint 2 and joint 3
    % When Joint 2 is at position x, Joint 3 range is [x-0.93, x+0.95] (more restrictive)
    q3_min_rel = q2 - 0.93;
    q3_max_rel = q2 + 0.95;
    
    % Apply relationship limits to q3
    q3_min_rel = max(q3_min_rel, q3_min);
    q3_max_rel = min(q3_max_rel, q3_max);
    q3 = max(q3_min_rel, min(q3_max_rel, q3));
    
    % Check if original values were within limits
    valid = true;
    if q1_orig < q1_min || q1_orig > q1_max
        valid = false;
        warning('Joint 1 clamped from %.4f to %.4f (limits: [%.4f, %.4f])', q1_orig, q1, q1_min, q1_max);
    end
    
    if q2_orig < q2_min || q2_orig > q2_max
        valid = false;
        warning('Joint 2 clamped from %.4f to %.4f (limits: [%.4f, %.4f])', q2_orig, q2, q2_min, q2_max);
    end
    
    if q3_orig < q3_min || q3_orig > q3_max
        valid = false;
        warning('Joint 3 clamped from %.4f to %.4f (limits: [%.4f, %.4f])', q3_orig, q3, q3_min, q3_max);
    end
    
    % Check relationship limits
    if q3_orig < q3_min_rel || q3_orig > q3_max_rel
        valid = false;
        warning('Joint 3 clamped from %.4f to %.4f (relationship limits: [%.4f, %.4f] for q2=%.4f)', q3_orig, q3, q3_min_rel, q3_max_rel, q2);
    end
    
    % Calculate forward kinematics with clamped values
    x=sin(q1).*(l1*cos(q2)+l2*sin(q3));
    y=l2-l2*cos(q3)+l1*sin(q2);
    z=-l1+cos(q1).*(l1*cos(q2)+l2*sin(q3));
end