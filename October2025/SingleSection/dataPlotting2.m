close all
% publish('plottingData.m', 'html');

for i = 1:2
    % Filenames
    PdataFile = sprintf('Pdata%d.mat', i);
    SdataFile = sprintf('Sdata%d.mat', i);
    GdataFile = sprintf('Gdata%d.mat', i);
    
    % File existence
    hasPdata = exist(PdataFile, 'file');
    hasSdata = exist(SdataFile, 'file');
    hasGdata = exist(GdataFile, 'file');
    
    if hasPdata || hasSdata || hasGdata
        % Load what exists
        if hasPdata
            Pdata = load(PdataFile); fprintf('Loaded: %s\n', PdataFile);
        else
            Pdata = []; fprintf('Missing: %s\n', PdataFile);
        end
        if hasSdata
            Sdata = load(SdataFile); fprintf('Loaded: %s\n', SdataFile);
        else
            Sdata = []; fprintf('Missing: %s\n', SdataFile);
        end
        if hasGdata
            Gdata = load(GdataFile); fprintf('Loaded: %s\n', GdataFile);
        else
            Gdata = []; fprintf('Missing: %s\n', GdataFile);
        end
        
        % Plot
        plotPhantomSimulation(Pdata, Sdata, Gdata, sprintf('DataSet%d', i));
    else
        fprintf('Skipping dataset %d - no data files exist\n', i);
    end
end


function plotPhantomSimulation(Pdata, Sdata, Gdata, figPrefix)
    %% ------- Flags -------
    hasPdata = ~isempty(Pdata);
    hasSdata = ~isempty(Sdata);
    hasGdata = ~isempty(Gdata);
    
    %% ------- Extract / guard data -------
    % Phantom
    if hasPdata
        PqDes = Pdata.Pdata(:,1:3);
        PqdDes = Pdata.Pdata(:,4:6);
        PqAct = Pdata.Pdata(:,7:9);
        PqdAct = Pdata.Pdata(:,10:12);
    else
        PqDes=[]; PqdDes=[]; PqAct=[]; PqdAct=[];
    end
    
    % Simulation
    if hasSdata
        Stime = Sdata.tOpt;
        SqDes = Sdata.yOpt(:,1:3);
        SqdDes = Sdata.yOpt(:,4:6);
        SqAct = Sdata.yOpt(:,7:9);
        SqdAct = Sdata.yOpt(:,10:12);
        xTarget = Sdata.xTarget;
        if ~isempty(PqAct)
            Ptime = linspace(0, Stime(end), size(PqAct,1));
        else
            Ptime = [];
        end
    else
        Stime=[]; SqDes=[]; SqdDes=[]; SqAct=[]; SqdAct=[]; xTarget=[];
        Ptime=[];
    end
    
    % Gain
    if hasGdata
        Gtime = Gdata.Hys;
        GqDes = Gdata.yOut1(:,1:3);
        GqdDes = Gdata.yOut1(:,4:6);
        GqAct = Gdata.yOut1(:,7:9);
        GqdAct = Gdata.yOut1(:,10:12);
    else
        Gtime=[]; GqDes=[]; GqdDes=[]; GqAct=[]; GqdAct=[];
    end
    
    %% ------- Forward kinematics (if available) -------
    % Simulation FK
    if ~isempty(SqDes), [SxDes,SyDes,SzDes] = FK(SqDes(:,1),SqDes(:,2),SqDes(:,3)); else, SxDes=[];SyDes=[];SzDes=[]; end
    if ~isempty(SqAct), [SxAct,SyAct,SzAct] = FK(SqAct(:,1),SqAct(:,2),SqAct(:,3)); else, SxAct=[];SyAct=[];SzAct=[]; end
    % Phantom FK
    if ~isempty(PqAct), [PxAct,PyAct,PzAct] = FK(PqAct(:,1),PqAct(:,2),PqAct(:,3)); else, PxAct=[];PyAct=[];PzAct=[]; end
    % Gain FK
    if ~isempty(GqDes), [GxDes,GyDes,GzDes] = FK(GqDes(:,1),GqDes(:,2),GqDes(:,3)); else, GxDes=[];GyDes=[];GzDes=[]; end
    if ~isempty(GqAct), [GxAct,GyAct,GzAct] = FK(GqAct(:,1),GqAct(:,2),GqAct(:,3)); else, GxAct=[];GyAct=[];GzAct=[]; end
    
    %% ------- Distances to targets (for legend text) -------
    if ~isempty(xTarget)
        nTargets = size(xTarget,1);
        TargetMinSim  = zeros(nTargets,1);
        TargetMinPthm = zeros(nTargets,1);
        TargetMinGain = zeros(nTargets,1);
        for k = 1:nTargets
            if ~isempty(SxAct)
                diffsSim = [SxAct,SyAct,SzAct] - xTarget(k,:);
                TargetMinSim(k) = min(vecnorm(diffsSim,2,2));
            end
            if ~isempty(PxAct)
                diffsPthm = [PxAct,PyAct,PzAct] - xTarget(k,:);
                TargetMinPthm(k) = min(vecnorm(diffsPthm,2,2));
            end
            if ~isempty(GxAct)
                diffsGain = [GxAct,GyAct,GzAct] - xTarget(k,:);
                TargetMinGain(k) = min(vecnorm(diffsGain,2,2));
            end
        end
    else
        nTargets=0; TargetMinSim=[]; TargetMinPthm=[]; TargetMinGain=[];
    end
    
    %% ==================== ONE FIGURE: 3D TOP + 2D BOTTOM (XY, XZ, YZ) ====================
    figAll = figure('Name',[figPrefix ' - 3D + Projections']);
    tl = tiledlayout(figAll, 2, 3, 'Padding','compact','TileSpacing','compact');

    % ---------- 3D AXIS (top row spans all 3 columns) ----------
    ax3d = nexttile(tl, 1, [1 3]); hold(ax3d,'on'); grid(ax3d,'on'); view(ax3d,3); axis(ax3d,'equal');
    xlabel(ax3d,'$X$ (m)','Interpreter','latex'); ylabel(ax3d,'$Y$ (m)','Interpreter','latex'); zlabel(ax3d,'$Z$ (m)','Interpreter','latex');
    title(ax3d,'3D Cartesian Space Position');

    plotHandles = gobjects(0); plotLabels = {};

    % Start (origin)
    hStart = plot3(ax3d,0,0,0,'o','LineWidth',1.2,'MarkerSize',6,'DisplayName','Start Point');
    plotHandles(end+1)=hStart; plotLabels{end+1}='Start Point';

    % Final point
    if hasSdata && isfield(Sdata,'xFinal') && numel(Sdata.xFinal)>=3
        xf = Sdata.xFinal(:).';
        hFinal = plot3(ax3d, xf(1),xf(2),xf(3), 'o','LineWidth',1.2,'MarkerSize',6,'DisplayName','Final Point');
        plotHandles(end+1)=hFinal; plotLabels{end+1}='Final Point';
    end

    % Reference / Trajectories
    if ~isempty(SxDes)
        hDes = plot3(ax3d, SxDes,SyDes,SzDes,'k--','LineWidth',2,'DisplayName','Reference Position ($q_d$)');
        plotHandles(end+1)=hDes; plotLabels{end+1}='Reference Position ($q_d$)';
    end
    if ~isempty(SxAct)
        hSim = plot3(ax3d, SxAct,SyAct,SzAct,'r-','LineWidth',2,'DisplayName','Simulation Position ($q_{\mathrm{sim}}$)');
        plotHandles(end+1)=hSim; plotLabels{end+1}='Simulation Position ($q_{\mathrm{sim}}$)';
    end
    if ~isempty(PxAct)
        hPh = plot3(ax3d, PxAct,PyAct,PzAct,'g-','LineWidth',2,'DisplayName','Phantom Position ($q_{\mathrm{ph}}$)');
        plotHandles(end+1)=hPh; plotLabels{end+1}='Phantom Position ($q_{\mathrm{ph}}$)';
    end
    if ~isempty(GxAct)
        hGain = plot3(ax3d, GxAct,GyAct,GzAct,'m-','LineWidth',2,'DisplayName','Gain Trajectory');
        plotHandles(end+1)=hGain; plotLabels{end+1}='Gain Trajectory';
    end

    % Targets
    if nTargets>0
        tgtH = gobjects(nTargets,1); tgtL = cell(nTargets,1);
        for k=1:nTargets
            tgtH(k)=plot3(ax3d, xTarget(k,1),xTarget(k,2),xTarget(k,3),'p','MarkerSize',12,...
                'MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName',sprintf('Target %d',k));
            tgtL{k}=sprintf('Target %d',k);
        end
        plotHandles=[plotHandles, tgtH(:)']; plotLabels=[plotLabels, tgtL(:)'];
    end

    % Error rows (legend only)
    errH = gobjects(0); errL = {};
    if nTargets>0
        for k=1:nTargets
            if ~isempty(TargetMinSim),  errH(end+1)=plot(ax3d,nan,nan,'w'); errL{end+1}=sprintf('Target %d Sim Err: %.6f', k, TargetMinSim(k));  end
            if ~isempty(TargetMinPthm), errH(end+1)=plot(ax3d,nan,nan,'w'); errL{end+1}=sprintf('Target %d Pthm Err: %.6f', k, TargetMinPthm(k)); end
            if ~isempty(TargetMinGain), errH(end+1)=plot(ax3d,nan,nan,'w'); errL{end+1}=sprintf('Target %d Gain Err: %.6f', k, TargetMinGain(k)); end
        end
    end

    % ---------- Build trajectory sets for projections ----------
    trajs = struct('X',{},'Y',{},'Z',{},'spec',{},'lw',{},'label',{});
    addT = @(X,Y,Z,spec,lw,label) struct('X',X,'Y',Y,'Z',Z,'spec',spec,'lw',lw,'label',label);
    if ~isempty(SxDes), trajs(end+1)=addT(SxDes,SyDes,SzDes,'k--',2,'Reference ($q_d$)'); end
    if ~isempty(SxAct), trajs(end+1)=addT(SxAct,SyAct,SzAct,'r-',2,'Simulation ($q_{\mathrm{sim}}$)'); end
    if ~isempty(PxAct), trajs(end+1)=addT(PxAct,PyAct,PzAct,'g-',2,'Phantom ($q_{\mathrm{ph}}$)'); end
    if ~isempty(GxAct), trajs(end+1)=addT(GxAct,GyAct,GzAct,'m-',2,'Gain Trajectory'); end

    % Limits helper
    function [xlimv,ylimv] = computeLimits(Xcells, Ycells, tgtXY)
        xmin= inf; xmax=-inf; ymin= inf; ymax=-inf;
        for ii=1:numel(Xcells)
            if isempty(Xcells{ii}) || isempty(Ycells{ii}), continue; end
            xmin=min(xmin, min(Xcells{ii})); xmax=max(xmax, max(Xcells{ii}));
            ymin=min(ymin, min(Ycells{ii})); ymax=max(ymax, max(Ycells{ii}));
        end
        if ~isempty(tgtXY)
            xmin=min(xmin, min(tgtXY(:,1))); xmax=max(xmax, max(tgtXY(:,1)));
            ymin=min(ymin, min(tgtXY(:,2))); ymax=max(ymax, max(tgtXY(:,2)));
        end
        if ~isfinite(xmin) || ~isfinite(xmax) || ~isfinite(ymin) || ~isfinite(ymax)
            xlimv=[-1 1]; ylimv=[-1 1]; return;
        end
        span=max([xmax-xmin, ymax-ymin, eps]); pad=0.05*span;
        xlimv=[xmin-pad, xmax+pad]; ylimv=[ymin-pad, ymax+pad];
    end

    % Build arrays for limits
    X_xy = cellfun(@(i) trajs(i).X, num2cell(1:numel(trajs)), 'UniformOutput', false);
    Y_xy = cellfun(@(i) trajs(i).Y, num2cell(1:numel(trajs)), 'UniformOutput', false);
    X_xz = cellfun(@(i) trajs(i).X, num2cell(1:numel(trajs)), 'UniformOutput', false);
    Z_xz = cellfun(@(i) trajs(i).Z, num2cell(1:numel(trajs)), 'UniformOutput', false);
    Y_yz = cellfun(@(i) trajs(i).Y, num2cell(1:numel(trajs)), 'UniformOutput', false);
    Z_yz = cellfun(@(i) trajs(i).Z, num2cell(1:numel(trajs)), 'UniformOutput', false);

    tgtXY = []; tgtXZ = []; tgtYZ = [];
    if ~isempty(xTarget)
        tgtXY = xTarget(:,[1 2]);
        tgtXZ = xTarget(:,[1 3]);
        tgtYZ = xTarget(:,[2 3]);
    end
    [xlim_xy, ylim_xy] = computeLimits(X_xy, Y_xy, tgtXY);
    [xlim_xz, ylim_xz] = computeLimits(X_xz, Z_xz, tgtXZ);
    [xlim_yz, ylim_yz] = computeLimits(Y_yz, Z_yz, tgtYZ);

    % Drawer for a plane
    function drawPlane(ax, Xs, Ys, labX, labY, limX, limY, projTag)
        hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
        for jj=1:numel(trajs)
            plot(ax, Xs{jj}, Ys{jj}, trajs(jj).spec, 'LineWidth', trajs(jj).lw, 'DisplayName', trajs(jj).label);
        end
        % Start
        plot(ax, 0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 1.0, 'DisplayName','Start Point');
        % Final point if present
        if hasSdata && isfield(Sdata,'xFinal') && numel(Sdata.xFinal)>=3
            xf=Sdata.xFinal(:).';
            switch projTag
                case "XY", plot(ax, xf(1), xf(2), 'o','MarkerSize',6,'LineWidth',1.0,'Color',[0.85 0.33 0.10],'DisplayName','Final Point');
                case "XZ", plot(ax, xf(1), xf(3), 'o','MarkerSize',6,'LineWidth',1.0,'Color',[0.85 0.33 0.10],'DisplayName','Final Point');
                case "YZ", plot(ax, xf(2), xf(3), 'o','MarkerSize',6,'LineWidth',1.0,'Color',[0.85 0.33 0.10],'DisplayName','Final Point');
            end
        end
        % Targets
        if ~isempty(xTarget)
            switch projTag
                case "XY", plot(ax, xTarget(:,1), xTarget(:,2), 'p', 'MarkerSize', 9, 'MarkerFaceColor','k','MarkerEdgeColor','k', 'DisplayName','Target');
                case "XZ", plot(ax, xTarget(:,1), xTarget(:,3), 'p', 'MarkerSize', 9, 'MarkerFaceColor','k','MarkerEdgeColor','k', 'DisplayName','Target');
                case "YZ", plot(ax, xTarget(:,2), xTarget(:,3), 'p', 'MarkerSize', 9, 'MarkerFaceColor','k','MarkerEdgeColor','k', 'DisplayName','Target');
            end
        end
        xlim(ax, limX); ylim(ax, limY);
        xlabel(ax, labX + " (m)", 'Interpreter','latex');
        ylabel(ax, labY + " (m)", 'Interpreter','latex');
        ax.FontName='Times New Roman'; ax.FontSize=10; ax.TickDir='out';
    end

    % ---------- Bottom row: XY / XZ / YZ ----------
    ax_xy = nexttile(tl, 4);  % (row 2, col 1)
    drawPlane(ax_xy, X_xy, Y_xy, "$X$", "$Y$", xlim_xy, ylim_xy, "XY");
    title(ax_xy, 'XY Projection');

    ax_xz = nexttile(tl, 5);  % (row 2, col 2)
    drawPlane(ax_xz, X_xz, Z_xz, "$X$", "$Z$", xlim_xz, ylim_xz, "XZ");
    title(ax_xz, 'XZ Projection');

    ax_yz = nexttile(tl, 6);  % (row 2, col 3)
    drawPlane(ax_yz, Y_yz, Z_yz, "$Y$", "$Z$", xlim_yz, ylim_yz, "YZ");
    title(ax_yz, 'YZ Projection');

    % ---------- Legend outside on the right (only include what's present) ----------
    proxyH = gobjects(0); proxyL = {};
    
    % Start point (always)
    proxyH(end+1) = plot(ax3d, nan,nan,'o','LineWidth',1.2,'MarkerSize',6, ...
        'DisplayName','Start Point');                  proxyL{end+1} = 'Start Point';
    
    % Final point (only if available)
    if hasSdata && isfield(Sdata,'xFinal') && numel(Sdata.xFinal)>=3
        proxyH(end+1) = plot(ax3d, nan,nan,'o','LineWidth',1.2,'MarkerSize',6, ...
            'DisplayName','Final Point');              proxyL{end+1} = 'Final Point';
    end
    
    % Reference ($q_d$)
    if ~isempty(SxDes)
        proxyH(end+1) = plot(ax3d, nan,nan,'k--','LineWidth',2, ...
            'DisplayName','Reference Position ($q_d$)');
        proxyL{end+1} = 'Reference Position ($q_d$)';
    end
    
    % Simulation ($q_{sim}$)
    if ~isempty(SxAct)
        proxyH(end+1) = plot(ax3d, nan,nan,'r-','LineWidth',2, ...
            'DisplayName','Simulation Position ($q_{\mathrm{sim}}$)');
        proxyL{end+1} = 'Simulation Position ($q_{\mathrm{sim}}$)';
    end
    
    % Phantom ($q_{ph}$)
    if ~isempty(PxAct)
        proxyH(end+1) = plot(ax3d, nan,nan,'g-','LineWidth',2, ...
            'DisplayName','Phantom Position ($q_{\mathrm{ph}}$)');
        proxyL{end+1} = 'Phantom Position ($q_{\mathrm{ph}}$)';
    end
    
    % Gain trajectory (ONLY if exists)
    if ~isempty(GxAct)
        proxyH(end+1) = plot(ax3d, nan,nan,'m-','LineWidth',2, ...
            'DisplayName','Gain Trajectory');
        proxyL{end+1} = 'Gain Trajectory';
    end
    
    % Targets (only if present)
    if nTargets > 0
        proxyH(end+1) = plot(ax3d, nan,nan,'p','MarkerSize',10, ...
            'MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName','Target');
        proxyL{end+1} = 'Target';
    end
    
    % Append error “rows” (if any)
    allH = [proxyH, errH];
    allL = [proxyL,  errL];
    
    % Create legend on the 3D axes and dock to the right
    lgd = legend(ax3d, allH, allL, 'Interpreter','latex');
    try
        lgd.Layout.Tile = 'east';     % R2020b+ (docks to the right of tiledlayout)
    catch
        lgd.Location = 'eastoutside'; % Fallback for older MATLAB
    end



    %% ==================== Joint Position ====================
    if ~isempty(PqAct) || ~isempty(SqDes) || ~isempty(GqAct)
        fig2 = figure('Name',[figPrefix ' - Joint Position']);
        for j=1:3
            subplot(3,1,j); grid on; hold on;
            sh = gobjects(0); sl = {};
            if ~isempty(PqAct)
                hP = plot(Ptime, PqAct(:,j), 'r-','LineWidth',1.2); sh(end+1)=hP; sl{end+1}='Phantom Actual';
            end
            if ~isempty(SqDes)
                hSd = plot(Stime, SqDes(:,j), 'k--','LineWidth',1.2); sh(end+1)=hSd; sl{end+1}=sprintf('Reference ($q_{d,%d}$)',j);
            end
            if ~isempty(SqAct)
                hSa = plot(Stime, SqAct(:,j), 'b-','LineWidth',1.2); sh(end+1)=hSa; sl{end+1}=sprintf('Simulation ($q_{%d}$)',j);
            end
            if ~isempty(GqAct)
                hG = plot(Gtime, GqAct(:,j), 'm-','LineWidth',1.2); sh(end+1)=hG; sl{end+1}='Gain Actual';
            end
            title(sprintf('Joint %d Position', j));
            if j==3, xlabel('Time (s)'); end
            ylabel('Position (rad)');
            legend(sh, sl, 'Interpreter','latex', 'Location','best');
        end
    end
    
    %% ==================== Joint Velocity ====================
    if ~isempty(PqdAct) || ~isempty(SqdDes) || ~isempty(GqdAct)
        fig3 = figure('Name',[figPrefix ' - Joint Velocity']);
        for j=1:3
            subplot(3,1,j); grid on; hold on;
            sh = gobjects(0); sl = {};
            if ~isempty(PqdAct)
                hP = plot(Ptime, PqdAct(:,j), 'r-','LineWidth',1.2); sh(end+1)=hP; sl{end+1}='Phantom Actual';
            end
            if ~isempty(SqdDes)
                hSd = plot(Stime, SqdDes(:,j), 'k--','LineWidth',1.2); sh(end+1)=hSd; sl{end+1}=sprintf('Reference ($\\dot{q}_{d,%d}$)',j);
            end
            if ~isempty(SqdAct)
                hSa = plot(Stime, SqdAct(:,j), 'b-','LineWidth',1.2); sh(end+1)=hSa; sl{end+1}=sprintf('Simulation ($\\dot{q}_{%d}$)',j);
            end
            if ~isempty(GqdAct)
                hG = plot(Gtime, GqdAct(:,j), 'm-','LineWidth',1.2); sh(end+1)=hG; sl{end+1}='Gain Actual';
            end
            title(sprintf('Joint %d Velocity', j));
            if j==3, xlabel('Time (s)'); end
            ylabel('Velocity (rad/s)');
            legend(sh, sl, 'Interpreter','latex', 'Location','best');
        end
    end
    
    % Optional saves:
    % exportgraphics(figAll, sprintf('%s_3D_and_Projections.pdf', figPrefix), 'ContentType','vector');
    % exportgraphics(fig2, sprintf('%s_Joint_Position.pdf', figPrefix), 'ContentType','vector');
    % exportgraphics(fig3, sprintf('%s_Joint_Velocity.pdf', figPrefix), 'ContentType','vector');
end
