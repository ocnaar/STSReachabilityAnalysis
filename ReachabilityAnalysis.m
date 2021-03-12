%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Spring 2018                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Compute over-approximations for the reachable sets of the state, input, %
% and output of a three-link planar robot used to model the sit-to-stand  %
% (STS) movement of a Powered Lower Limb Orthosis (PLLO) in the presence  %
% of constant parameter uncertainty, and starting from a single initial   %
% state.                                                                  %
%                                                                         %
% The sensitivity-based approach is published in:                         %
%                                                                         %
% O. Narváez-Aroche, P. J. Meyer, M. Arcak, A. Packard, “Reachability     %
% analysis for robustness evaluation of the Sit-To-Stand movement for     %
% powered lower limb orthoses”, Proceedings of ASME Dynamic Systems and   %
% Control Conference, October 2018.                                       %
% https://doi.org/10.1115/DSCC2018-9066                                   %
%                                                                         %
% Please cite our work accordingly.                                       %
%                                                                         %
% Install the LTV toolbox available at https://z.umn.edu/LTVTools         %
% and Matlab's Parallel Computing Toolbox before running this script.     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set a clean workspace
clear
clc
close all

%% Start parallel pool of workers, using the default cluster profile.
parpool();

%% Load data for sample-based reachability analysis.
load('ReachData.mat');

%% Solve sensitivity function with respect to parameters along nominal trajectory. 

% Start timer. 
tStart = tic;

fprintf('\nSolve sensitivity equation along reference trajectories, and nominal parameters...\n')
tic
[TS,Sens] = ode45(@(t,s) TVLQRSTS3LinkSensitivityPvec(t,s,tgrid,xbar1,ubar1,K,pnom,epsilon,err,maxiter),tgrid,[th1(1:6); zeros(nx*numel(pnom),1)]);
toc

% Retrieve sensitivity data from simulation.
Sens = Sens(:,nx+1:end);

% Array for storing parameter sensitivity about nominal trajectory.
S_nom = zeros(nx,np,nT);

% Interpolate parameter sensitivity of the nominal trajectory at sampling points. 
for i = 1:nx
    for j = 1:np
        idx = (i-1)*np+j;
        S_nom(i,j,:) = csapi(TS,Sens(:,idx),tgrid);
    end
end

%% Sampling-based method to obtain bounds on the state sensitivity matrix entries over the set of parameters.

% Number of samples to compute bounds. 
sample_number = 500;

% Array for storing random samples.
par_random = zeros(sample_number,np);

% Latin hypercube sampling of uncertain parameters. 
LHS = lhsdesign(sample_number,np);
for i = 1:sample_number
    par_random(i,:) = pmin + LHS(i,:).*(pmax-pmin);
end

% Cell arrays for storing simulations of sampled parameters. 
T_Sens_rand = cell(1,sample_number);
Sens_rand = cell(1,sample_number);

% Compute sensitivities with sampled parameters.
fprintf('\nCompute sensitivities for the evolution of the states with %d parameter samples...\n',sample_number)
tic
parfor i = 1:sample_number
    [T_Sens_rand{i},Sens_rand{i}] = ode45(@(t,s) TVLQRSTS3LinkSensitivityPvec(t,s,tgrid,xbar1,ubar1,K,par_random(i,:),epsilon,err,maxiter),tgrid,[th1(1:6); zeros(nx*np,1)]);
    Sens_rand{i} = Sens_rand{i}(:,nx+1:end);
end
toc

% Cell array for storing interpolation of sensitivities for sampled parameters.
S_rand = cell(1,sample_number);

% Interpolate parameter sensitivity of the simulations at sampling points.
fprintf('\nInterpolate state sensitivities for the %d parameter samples...\n', sample_number)
tic
for k = 1:sample_number
    S_rand{k} = zeros(nx,np,nT);
    for i = 1:nx
        for j = 1:np
            idx = (i-1)*np+j;
            S_rand{k}(i,j,:) = csapi(T_Sens_rand{k},Sens_rand{k}(:,idx),tgrid);
        end
    end
end
toc

% Arrays for storing sensitivity bounds.
Sensitivity_LB = S_nom;
Sensitivity_UB = S_nom;

% Compute state sensitivity bounds.
fprintf('\nCompute state sensitivity bounds from the simulations for %d parameter samples...\n', sample_number)
tic
for i=1:nT
    for j=1:sample_number
        Sensitivity_LB(:,:,i) = min(Sensitivity_LB(:,:,i),S_rand{j}(:,:,i));
        Sensitivity_UB(:,:,i) = max(Sensitivity_UB(:,:,i),S_rand{j}(:,:,i));
    end
end
toc

%% State and compensation vectors for computation of state over-approximations, based on sensitivity bounds.

% Arrays for storing initial conditions
z = zeros(nx,np,nT);
y = zeros(nx,np,nT);
alpha = zeros(nx,np,nT);

fprintf('\nCompute compensation vectors based on state sensitivity bounds...\n')
tic
for k=1:nT
    for i = 1:nx
        for j = 1:np
            if Sensitivity_LB(i,j,k) >= 0                         % positive
                z(i,j,k) = pmin(j);
                y(i,j,k) = pmax(j);
            elseif Sensitivity_UB(i,j,k) <= 0                     % negative
                z(i,j,k) = pmax(j);
                y(i,j,k) = pmin(j);
            elseif -Sensitivity_LB(i,j,k) <= Sensitivity_UB(i,j,k)% mostly positive
                z(i,j,k) = pmin(j);
                y(i,j,k) = pmax(j);
                alpha(i,j,k) = Sensitivity_LB(i,j,k);
            else                                                  % mostly negative
                z(i,j,k) = pmax(j);
                y(i,j,k) = pmin(j);
                alpha(i,j,k) = Sensitivity_UB(i,j,k);
            end
        end
    end
end
toc

%% Sensitivity-based state over-approximations

% Arrays for storing lower and upper bounds.
xlbnd = zeros(nT,nx);
xubnd = zeros(nT,nx);

% Add initial condition to bounds. 
xlbnd(1,:) = xbar1(1,:);
xubnd(1,:) = xbar1(1,:);

% Cells for performing parallel simulations.
z_traj = cell(1,nx);
y_traj = cell(1,nx);

% Compute bounds overapproximatting the STS movement reachable states tunnel.
fprintf('\nCompute upper and lower bounds over-approximatting the tunnel of states for the STS movement under parameter uncertainty...\n')
tic
for k = 2:nT
    parfor i = 1:nx
        % Trajectory from vector of parameters z.
        [~,z_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,z(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        
        % Trajectory from vector of parameters y.
        [~,y_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,y(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        
        % Over-approximation on dimension i at time tgrid(k)
        xlbnd(k,i) = z_traj{i}(end,i)-alpha(i,:,k)*(z(i,:,k)-y(i,:,k))';
        xubnd(k,i) = y_traj{i}(end,i)+alpha(i,:,k)*(z(i,:,k)-y(i,:,k))';
    end
end
toc

%% Find bounds on input sensitivity with respect to parameters via trajectory sampling

% Arrays for storing input sensitivity bounds.
U_Sensitivity_LB = zeros(nu,np,nT);
U_Sensitivity_UB = zeros(nu,np,nT);

for i = 1:nT
    Kt = eval2obj(K,tgrid(i));
    U_Sensitivity_LB(:,:,i) = -Kt.MatArray*S_nom(:,:,i);
    U_Sensitivity_UB(:,:,i) = -Kt.MatArray*S_nom(:,:,i);
end

% Compute input sensitivity bounds.
fprintf('\nCompute input sensitivity bounds from the simulations of %d parameter samples...\n',sample_number)
tic
for i=1:nT
    Kt = eval2obj(K,tgrid(i));
    for j=1:sample_number
        U_Sensitivity_LB(:,:,i) = min(U_Sensitivity_LB(:,:,i),-Kt.MatArray*S_rand{j}(:,:,i));
        U_Sensitivity_UB(:,:,i) = max(U_Sensitivity_UB(:,:,i),-Kt.MatArray*S_rand{j}(:,:,i));
    end
end
toc

%% State and compensation vectors for computation of input over-approximation, based on sensitivity bounds 

% Arrays for storing parameters for simulation. 
u_z = zeros(nu,np,nT);
u_y = zeros(nu,np,nT);
u_alpha = zeros(nu,np,nT);

fprintf('\nCompute compensation vectors based on input sensitivity bounds...\n')
tic
for k = 1:nT
    for i = 1:nu
        for j = 1:np
            if U_Sensitivity_LB(i,j,k) >= 0                            % positive
                u_z(i,j,k) = pmin(j);
                u_y(i,j,k) = pmax(j);
            elseif U_Sensitivity_UB(i,j,k) <= 0                        % negative
                u_z(i,j,k) = pmax(j);
                u_y(i,j,k) = pmin(j);
            elseif -U_Sensitivity_LB(i,j,k) <= U_Sensitivity_UB(i,j,k) % mostly positive
                u_z(i,j,k) = pmin(j);
                u_y(i,j,k) = pmax(j);
                u_alpha(i,j,k) = U_Sensitivity_LB(i,j,k);
            else                                                       % mostly negative
                u_z(i,j,k) = pmax(j);
                u_y(i,j,k) = pmin(j);
                u_alpha(i,j,k) = U_Sensitivity_UB(i,j,k);
            end
        end
    end
end
toc

%% Sensitivity-based input over-approximation

% Arrays for storing lower and upper bounds.
ulbnd = zeros(nT,nu);
uubnd = zeros(nT,nu);

% Add initial condition to bounds. 
ulbnd(1,:) = ubar1(1,:);
uubnd(1,:) = ubar1(1,:);

% Cells for performing parallel simulations.
u_z_traj = cell(1,nu);
u_y_traj = cell(1,nu);

% Compute bounds overapproximatting the STS movement reachable input tunnel.
fprintf('\nCompute upper and lower bounds over-approximatting the tunnel of inputs for the STS movement under parameter uncertainty...\n')
tic
for k = 2: nT
    parfor i = 1:nu
        % Trajectory from vector of parameters z.
        [~,u_z_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,u_z(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        u_z_traj{i} = TVLQRThreeLinkInputs(tgrid(k),u_z_traj{i}(end,:),K,tgrid,xbar1,ubar1);
        
        % Trajectory from vector of parameters y.
        [~,u_y_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,u_y(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        u_y_traj{i} = TVLQRThreeLinkInputs(tgrid(k),u_y_traj{i}(end,:),K,tgrid,xbar1,ubar1);
        
        % Over-approximation on dimension i at time tgrid(k)
        ulbnd(k,i) = u_z_traj{i}(end,i)-u_alpha(i,:,k)*(u_z(i,:,k)-u_y(i,:,k))';
        uubnd(k,i) = u_y_traj{i}(end,i)+u_alpha(i,:,k)*(u_z(i,:,k)-u_y(i,:,k))';
    end
end
toc

%% Find bounds on output sensitivity with respect to parameter uncertainty via trajectory sampling

% Array for storing output sensitivity bounds.
Z_Sensitivity_LB = zeros(nx,np,nT);

% Output sensitivity about nominal trajectory. 
[~,dgdx,dgdp] = x2zThreeLink(xbar1',pnom);
for i = 1:nT
    Z_Sensitivity_LB(:,:,i) = dgdx(:,:,i)*S_nom(:,:,i) + dgdp(:,:,i);
end
Z_Sensitivity_UB = Z_Sensitivity_LB;

% Compute output sensitivity bounds.
fprintf('\nCompute output sensitivity bounds from the simulations for %d parameter samples...\n',sample_number)
tic
for i=1:nT
    for j=1:sample_number
        Z_Sensitivity_rand = dgdx(:,:,i)*S_rand{j}(:,:,i) + dgdp(:,:,i); 
        Z_Sensitivity_LB(:,:,i) = min(Z_Sensitivity_LB(:,:,i),Z_Sensitivity_rand);
        Z_Sensitivity_UB(:,:,i) = max(Z_Sensitivity_UB(:,:,i),Z_Sensitivity_rand);
    end
end
toc

%% State and compensation vectors for computation of output over-approximation, based on sensitivity bounds 

% Arrays for storing parameters for simulation. 
z_z = zeros(nx,np,nT);
z_y = zeros(nx,np,nT);
z_alpha = zeros(nx,np,nT);

fprintf('\nCompute compensation vectors based on output sensitivity bounds...\n')
tic
for k = 1:nT
    for i = 1:nx
        for j = 1:np
            if Z_Sensitivity_LB(i,j,k) >= 0                            % positive
                z_z(i,j,k) = pmin(j);
                z_y(i,j,k) = pmax(j);
            elseif Z_Sensitivity_UB(i,j,k) <= 0                        % negative
                z_z(i,j,k) = pmax(j);
                z_y(i,j,k) = pmin(j);
            elseif -Z_Sensitivity_LB(i,j,k) <= Z_Sensitivity_UB(i,j,k) % mostly positive
                z_z(i,j,k) = pmin(j);
                z_y(i,j,k) = pmax(j);
                z_alpha(i,j,k) = Z_Sensitivity_LB(i,j,k);
            else                                                       % mostly negative
                z_z(i,j,k) = pmax(j);
                z_y(i,j,k) = pmin(j);
                z_alpha(i,j,k) = Z_Sensitivity_UB(i,j,k);
            end
        end
    end
end
toc

%% Sensitivity-based over-approximation of the output

% Arrays for storing lower and upper bounds.
zlbnd = zeros(nT,nx);
zubnd = zeros(nT,nx);

% Find initial bounds for the position of the CoM with fmincon. 
zlbnd(1,:) = [-pi/2, 0, 0, 0, 0, 0];
zubnd(1,:) = [-pi/2, 0, 0, 0, 0, 0];

% Maximum number of objective function evaluations. 
maxfuneval = 20000;

% fmincon solver options. 
optionsfmc = optimoptions('fmincon','MaxFunctionEvaluations',maxfuneval,'display','iter'); 

% Maximize x coordinate of the CoM.
xCoMmaxfh = @(dv) -[0 1 0 0 0 0]*x2zThreeLink(xbar1(1,:)',dv);
[~,xCoMmax] = fmincon(xCoMmaxfh,pnom,[],[],[],[],pmin,pmax,[],optionsfmc); 
zubnd(1,2) = -xCoMmax;

% Objective function to maximize y coordinate of the CoM.
yCoMmaxfh = @(dv) -[0 0 1 0 0 0]*x2zThreeLink(xbar1(1,:)',dv);
[~,yCoMmax] = fmincon(yCoMmaxfh,pnom,[],[],[],[],pmin,pmax,[],optionsfmc); 
zubnd(1,3) = -yCoMmax;

% Objective function to minimize x coordinate of the CoM.
xCoMminfh = @(dv) [0 1 0 0 0 0]*x2zThreeLink(xbar1(1,:)',dv);
[~,xCoMmin] = fmincon(xCoMminfh,pnom,[],[],[],[],pmin,pmax,[],optionsfmc); 
zlbnd(1,2) = xCoMmin;

% Objective function to minimize y coordinate of the CoM.
yCoMminfh = @(dv) [0 0 1 0 0 0]*x2zThreeLink(xbar1(1,:)',dv);
[~,yCoMmin] = fmincon(yCoMminfh,pnom,[],[],[],[],pmin,pmax,[],optionsfmc); 
zlbnd(1,3) = yCoMmin;

% Cells for performing parallel simulations.
z_z_traj = cell(1,nx);
z_y_traj = cell(1,nx);

% Compute bounds overapproximatting the STS movement reachable tunnel.
fprintf('\nCompute upper and lower bounds over-approximatting the tunnel of outputs for the STS movement under parameter uncertainty...\n')
tic
for k = 2: nT
    parfor i = 1:nx
        % Trajectory from vector of parameters z.
        [~,z_z_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,z_z(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        z_z_traj{i} = x2zThreeLink(z_z_traj{i}(end,:)',z_z(i,:,k))';
        
        % Trajectory from vector of parameters y.
        [~,z_y_traj{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,z_y(i,:,k)),[0 tgrid(k)],th1(1:6),options);
        z_y_traj{i} = x2zThreeLink(z_y_traj{i}(end,:)',z_y(i,:,k))';
        
        % Over-approximation on dimension i at time tgrid(k)
        zlbnd(k,i) = z_z_traj{i}(end,i)-z_alpha(i,:,k)*(z_z(i,:,k)-z_y(i,:,k))';
        zubnd(k,i) = z_y_traj{i}(end,i)+z_alpha(i,:,k)*(z_z(i,:,k)-z_y(i,:,k))';
    end
end
toc

% End timer.
tEnd = toc(tStart);
fprintf('\nSTS movement reachability analysis completed in %d [h].\n',tEnd/3600);

%% Perform Monte Carlo simulations to check over-approximations. 

% Choose number of random parameter sets (experiments).
nexp = 500;

% Array of random parameter values. 
pexp = zeros(nexp,np);

% Perform latin hypercube sampling. 
LHS = lhsdesign(nexp,np);
for i=1:nexp
    pexp(i,:) = pmin + LHS(i,:).*(pmax-pmin);
end 

% Cell arrays for storing simulations. 
Tcell = cell(1,nexp+1);
Xcell = cell(1,nexp+1);
Ucell = cell(1,nexp+1);

% Simulate closed-loop system.
fprintf('\nSimulating closed-loop system with %d different parameter values...\n',nexp)
tic
parfor i=1:nexp
    [Tcell{i}, Xcell{i}] = ode45(@(t,x) TVLQRSTS3LinkPvec(t,x,tgrid,xbar1,ubar1,K,pexp(i,:)),[tgrid(1) tgrid(end)],xbar1(1,:)',options);
    % Retrieve inputs from simulation.
    Ucell{i} = TVLQRThreeLinkInputs(Tcell{i},Xcell{i},K,tgrid,xbar1,ubar1);
end
toc

% Include nominal trajectories in cell arrays.
Tcell{end} = tgrid';
Xcell{end} = xbar1;
Ucell{end} = ubar1;

%% Plot computed state, input, and output bounds.

plot3LinkBounds(Tcell,Xcell,Ucell,[pexp; pnom],tgrid,xlbnd,xubnd,ulbnd,uubnd,zlbnd,zubnd,3,0,'Trajectories for $p \in \mathcal{P}_s$','Reference Trajectory','Bounds from $\underline{r^x}(t)$ and $\overline{r^x}(t)$','Bounds from $\underline{r^u}(t)$ and $\overline{r^u}(t)$','Bounds from $\underline{r^y}(t)$ and $\overline{r^y}(t)$');

%% Plot over-approximation boxes and successors.

% Number of over-approximation boxes to plot. 
nb = 3; 

% Uniformly spaced points in time for over-approximations.
tspan = linspace(tgrid(1),tgrid(end),nb); 

% Nearest neighbor in tgrid array for each value in tspan.
[~,tspanidx] = ismember(tspan,tgrid);

% Number of states. 
nx = size(xbar1,2);

% Number of inputs. 
nu = size(ubar1,2);

% Arrays for storing bounds.
Xbounds = zeros(nb,nx,2);
Ubounds = zeros(nb,nu,2);
Zbounds = zeros(nb,nx,2);

% Lower and upper bounds from previous computations. 
Xbounds(:,:,1) = xlbnd(tspanidx,:);
Xbounds(:,:,2) = xubnd(tspanidx,:);
Ubounds(:,:,1) = ulbnd(tspanidx,:);
Ubounds(:,:,2) = uubnd(tspanidx,:);
Zbounds(:,:,1) = zlbnd(tspanidx,:);
Zbounds(:,:,2) = zubnd(tspanidx,:);

% Make plots.
PlotOverAppBoxes(tspan,Xbounds,Ubounds,Zbounds,Tcell,Xcell,Ucell,pexp,pnom,3,0,'PC')