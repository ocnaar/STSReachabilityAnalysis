%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Compute the input commanded by a time-varying finite time horizon LQR   % 
% controller from the simulation data of the closed-loop dynamics.        %
%                                                                         %
% Input                                                                   %
% 	T: nt by 1 array of time samples.                                     %
% 	X: nt by 6 array of state samples from closed-loop simulation.        %
% 	  x(:,1): angular position of link 1 relative to the horizontal [rad].%
% 	  x(:,2): angular position of link 2 relative to link 1 in [rad].     % 
% 	  x(:,3): angular position of link 3 relative to link 2 in [rad].     %
% 	  x(:,4): angular velocity of link 1 in [rad/s].                      %
% 	  x(:,5): angular velocity of link 2 in [rad/s].                      %
% 	  x(:,6): angular velocity of link 3 in [rad/s].                      %
%   Ktvmat: LTV toolbox object with 4 by 6 time-varying LQR gain.         % 
% 	tgrid: n by 1 time array for interpolation of reference trajectories. %
% 	xref: n by 6 array of state reference trajectory.                     %
% 	uref: n by 4 array of input reference trajectory.                     %
% 	  uref(:,1): torque applied to link 3 at hip joint in [N.m].          %
% 	  uref(:,2): torque applied to link 3 at shoulder joint in [N.m].     %    
%     uref(:,3): horizontal force applied at shoulder joint in [N].       %
%     uref(:,4): vertical force applied at shoulder joint in [N].         %
%                                                                         %
% Output                                                                  %
% 	U: nt by 4 array with input commanded by the LQR controller.          %
% 	  U(:,1): torque commanded to link 3 at hip joint in [N.m].           %
% 	  U(:,2): torque commanded to link 3 at shoulder joint in [N.m].      %    
%     U(:,3): horizontal force commanded at shoulder joint in [N].        %
%     U(:,4): vertical force commanded at shoulder joint in [N].          %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function U = TVLQRThreeLinkInputs(T,X,Ktvmat,tgrid,xref,uref)

% Number of states.
nx = size(xref,2);

% Number of inputs.
nu = size(uref,2);

% Time samples in simulation results. 
nt = length(T);

% Interpolation of reference state trajectory.
Xbar = zeros(nt,nx);
for k=1:nx
    Xbar(:,k) = csapi(tgrid,xref(:,k),T);
end

% Interpolation of reference input trajectory.
Ubar = zeros(nt,nu);
for k=1:nu
    Ubar(:,k) = csapi(tgrid,uref(:,k),T);
end

% Retrieve LQR input from simulation results.
dx = X(:,1:nx)-Xbar;
U = zeros(nt,nu);
for k = 1:nt
    Kt = eval2obj(Ktvmat,T(k));
    du = -Kt.MatArray*dx(k,:)';
    if nu == 1
        U(k,:) = Ubar(k,:) + [du,0,0,0];
    elseif nu == 4
        U(k,:) = Ubar(k,:) + du';
    end
end