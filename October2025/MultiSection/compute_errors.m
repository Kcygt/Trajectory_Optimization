function R = compute_joint_rmse(datasetIdx)
% COMPUTE_JOINT_RMSE  Joint-space RMSE (position & velocity) + trajectory-to-target distances.
%   R = COMPUTE_JOINT_RMSE(i) loads Sdata<i>.mat and (optionally) Pdata<i>.mat,
%   computes:
%     • Joint position & velocity RMSE (Ref–Sim, Ref–Phantom)
%     • Distance-to-target time series for Reference / Simulation / Phantom
%     • Minimum distance to target (value, time, point) for each trajectory
%
% OUTPUT (kept from previous version + new target fields):
%   R.q_rmse_ref_sim (1x3)     position RMSE per joint [rad]
%   R.q_rmse_ref_ph  (1x3)
%   R.dq_rmse_ref_sim (1x3)    velocity RMSE per joint [rad/s]
%   R.dq_rmse_ref_ph  (1x3)
%   -- NEW: target distance results --
%   R.Target.xTarget (1x3)
%   R.Target.dist_ref (:), R.Target.dist_sim (:), R.Target.dist_ph (:)
%   R.Target.ref.dist,  R.Target.ref.t,  R.Target.ref.point (1x3)
%   R.Target.sim.dist,  R.Target.sim.t,  R.Target.sim.point (1x3)
%   R.Target.ph.dist,   R.Target.ph.t,   R.Target.ph.point  (1x3)

    if nargin < 1, datasetIdx = 1; end

    % ---------- Load Data ----------
    sFile = sprintf('Sdata%d.mat', datasetIdx);
    pFile = sprintf('Pdata%d.mat', datasetIdx);

    if exist(sFile, 'file') ~= 2
        error('Missing %s (required).', sFile);
    end
    S = load(sFile);

    hasP = exist(pFile, 'file') == 2;
    if hasP
        P = load(pFile);
    else
        P = [];
    end

    % ---------- Extract joint signals ----------
    q_ref  = S.yOpt(:,1:3);
    dq_ref = S.yOpt(:,4:6);
    q_sim  = S.yOpt(:,7:9);
    dq_sim = S.yOpt(:,10:12);
    tSim   = S.tOpt(:);

    if hasP
        q_ph  = P.Pdata(:,7:9);
        dq_ph = P.Pdata(:,10:12);
        tPh   = linspace(0, tSim(end), size(q_ph,1))';
        % Interpolate phantom onto simulation time base
        q_ph_i  = interp1(tPh, q_ph,  tSim, 'linear', 'extrap');
        dq_ph_i = interp1(tPh, dq_ph, tSim, 'linear', 'extrap');
    else
        q_ph_i  = [];
        dq_ph_i = [];
    end

    % ---------- Compute Joint Errors ----------
    e_q_sim  = q_ref - q_sim;     % position error (ref - sim)
    e_dq_sim = dq_ref - dq_sim;   % velocity error (ref - sim)

    if hasP
        e_q_ph  = q_ref - q_ph_i;    % position error (ref - ph)
        e_dq_ph = dq_ref - dq_ph_i;  % velocity error (ref - ph)
    else
        e_q_ph  = [];
        e_dq_ph = [];
    end

    % ---------- RMSE (per joint) ----------
    q_rmse_ref_sim  = sqrt(mean(e_q_sim.^2,  1));
    dq_rmse_ref_sim = sqrt(mean(e_dq_sim.^2, 1));

    if hasP
        q_rmse_ref_ph  = sqrt(mean(e_q_ph.^2,  1));
        dq_rmse_ref_ph = sqrt(mean(e_dq_ph.^2, 1));
    else
        q_rmse_ref_ph  = [NaN NaN NaN];
        dq_rmse_ref_ph = [NaN NaN NaN];
    end

    % ===================== NEW: Distance to Target =====================
    % Target (from S.xTarget if available, else default)
    if isfield(S, 'xTarget') && ~isempty(S.xTarget)
        xTarget = S.xTarget(1,:);
    else
        xTarget = [0.8 0.5 0.4];
    end

    % Cartesian trajectories via FK
    [Xr, Yr, Zr] = FK(q_ref(:,1), q_ref(:,2), q_ref(:,3));  % Reference
    [Xs, Ys, Zs] = FK(q_sim(:,1), q_sim(:,2), q_sim(:,3));  % Simulation

    if hasP
        [Xp, Yp, Zp] = FK(q_ph_i(:,1), q_ph_i(:,2), q_ph_i(:,3));  % Phantom (interpolated)
    else
        Xp = []; Yp = []; Zp = [];
    end

    X_ref = [Xr Yr Zr];
    X_sim = [Xs Ys Zs];
    if hasP, X_ph = [Xp Yp Zp]; else, X_ph = []; end

    % Distance to target (time series)
    dist_ref_ts = sqrt(sum((X_ref - xTarget).^2, 2));
    dist_sim_ts = sqrt(sum((X_sim - xTarget).^2, 2));
    if hasP
        dist_ph_ts  = sqrt(sum((X_ph  - xTarget).^2, 2));
    else
        dist_ph_ts  = [];
    end

    % Minimum distance to target (value, time, point)
    [d_ref, i_ref] = minDistToPoint(X_ref, xTarget);
    [d_sim, i_sim] = minDistToPoint(X_sim, xTarget);
    if hasP
        [d_ph,  i_ph ] = minDistToPoint(X_ph,  xTarget);
    else
        d_ph = NaN; i_ph = NaN;
    end

    % ---------- Display (kept + new) ----------
    fprintf('\n[Sdata%d] Joint RMSE (Position / Velocity)\n', datasetIdx);
    fprintf('-----------------------------------------\n');
    for j = 1:3
        fprintf('Joint %d:\n', j);
        fprintf('  Pos RMSE Ref–Sim = %.6f rad', q_rmse_ref_sim(j));
        if ~isnan(q_rmse_ref_ph(j)), fprintf(' | Ref–Ph = %.6f rad', q_rmse_ref_ph(j)); end
        fprintf('\n  Vel RMSE Ref–Sim = %.6f rad/s', dq_rmse_ref_sim(j));
        if ~isnan(dq_rmse_ref_ph(j)), fprintf(' | Ref–Ph = %.6f rad/s', dq_rmse_ref_ph(j)); end
        fprintf('\n');
    end

    fprintf('Minimum distance to target [%.3f %.3f %.3f]:\n', xTarget);
    fprintf('  Reference: %.6f m @ t = %.4f s, point = [%.6f %.6f %.6f]\n', ...
        d_ref, tSim(i_ref), X_ref(i_ref,1), X_ref(i_ref,2), X_ref(i_ref,3));
    fprintf('  Simulation: %.6f m @ t = %.4f s, point = [%.6f %.6f %.6f]\n', ...
        d_sim, tSim(i_sim), X_sim(i_sim,1), X_sim(i_sim,2), X_sim(i_sim,3));
    if ~isnan(d_ph)
        fprintf('  Phantom   : %.6f m @ t = %.4f s, point = [%.6f %.6f %.6f]\n', ...
            d_ph, tSim(i_ph), X_ph(i_ph,1), X_ph(i_ph,2), X_ph(i_ph,3));
    end

    % ---------- Store Results (kept + new) ----------
    R = struct();
    % Kept (joint RMSE)
    R.q_rmse_ref_sim  = q_rmse_ref_sim;
    R.q_rmse_ref_ph   = q_rmse_ref_ph;
    R.dq_rmse_ref_sim = dq_rmse_ref_sim;
    R.dq_rmse_ref_ph  = dq_rmse_ref_ph;

    % New (target distances)
    R.Target = struct();
    R.Target.xTarget  = xTarget;
    R.Target.dist_ref = dist_ref_ts;
    R.Target.dist_sim = dist_sim_ts;
    R.Target.dist_ph  = dist_ph_ts;

    R.Target.ref.dist   = d_ref;
    R.Target.ref.t      = tSim(i_ref);
    R.Target.ref.point  = X_ref(i_ref,:);

    R.Target.sim.dist   = d_sim;
    R.Target.sim.t      = tSim(i_sim);
    R.Target.sim.point  = X_sim(i_sim,:);

    if ~isnan(d_ph)
        R.Target.ph.dist   = d_ph;
        R.Target.ph.t      = tSim(i_ph);
        R.Target.ph.point  = X_ph(i_ph,:);
    else
        R.Target.ph.dist   = NaN;
        R.Target.ph.t      = NaN;
        R.Target.ph.point  = [NaN NaN NaN];
    end
end

% ===== local helper =====
function [dmin, idx] = minDistToPoint(X, xT)
    d2 = sum((X - xT).^2, 2);
    [d2min, idx] = min(d2);
    dmin = sqrt(d2min);
end
