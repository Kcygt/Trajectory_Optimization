function [x, y, z, valid] = FKnew(q1, q2, q3)
    % Joint limits
    q1_min = -1.5708; q1_max = 1.5708;
    q2_min = -1.07;   q2_max = 1.97;
    q3_min = -0.4;    q3_max = 2.92;
    
    % Link lengths
    l1 = 0.208; 
    l2 = 0.168;
    
    % Store original values for validation
    q1_orig = q1;
    q2_orig = q2;
    q3_orig = q3;
    
    % Clamp to absolute limits
    q1 = max(q1_min, min(q1_max, q1));
    q2 = max(q2_min, min(q2_max, q2));
    q3 = max(q3_min, min(q3_max, q3));
    
    % Relative limits for q3 based on q2
    q3_min_rel = max(q3_min, q2 - 0.93);
    q3_max_rel = min(q3_max, q2 + 0.95);
    
    % Apply relative limits
    q3 = max(q3_min_rel, min(q3_max_rel, q3));
    
    % Validity check (element-wise)
    valid = true(size(q1));
    valid = valid & (q1_orig >= q1_min & q1_orig <= q1_max);
    valid = valid & (q2_orig >= q2_min & q2_orig <= q2_max);
    valid = valid & (q3_orig >= q3_min & q3_orig <= q3_max);
    valid = valid & (q3_orig >= q3_min_rel & q3_orig <= q3_max_rel);
    
    % Forward kinematics (element-wise operations)
    x = sin(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
    y = l2 - l2 .* cos(q3) + l1 .* sin(q2);
    z = -l1 + cos(q1) .* (l1 .* cos(q2) + l2 .* sin(q3));
end