%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Dynamics of a three-link planar robot under time-varying finite time    %
% horizon LQR control, and matrix differential equation for computing the %
% sensitivity of the state trajectories with respect to the parameters.   % 
%                                                                         %  
% Input                                                                   %
%                                                                         %
% t: time.                                                                %
% x: state.                                                               %
% 	x(1): angular position of link 1 relative to the horizontal in [rad]. %
% 	x(2): angular position of link 2 relative to link 1 in [rad].         %
% 	x(3): angular position of link 3 relative to link 2 in [rad].         %
% 	x(4): angular velocity of link 1 in the inertial frame [rad/s].       %
% 	x(5): angular velocity of link 2 in the inertial frame [rad/s].       %
% 	x(6): angular velocity of link 3 in the inertial frame [rad/s].       %
% 	x(7:18): sensitivity of x(1) with respect to parameters.              %
% 	x(19:30): sensitivity of x(2) with respect to parameters.             %
% 	x(31:42): sensitivity of x(3) with respect to parameters.             %
% 	x(43:54): sensitivity of x(4) with respect to parameters.             %
% 	x(55:67): sensitivity of x(5) with respect to parameters.             %
% 	x(68:78): sensitivity of x(6) with respect to parameters.             %
% tgrid: n by 1 time array for interpolation of reference trajectories.   %
% xbar: n by 6 array of state reference trajectory.                       %
% ubar: n by 4 array of input reference trajectory.                       %
% 	ubar(:,1): torque applied to link 3 at hip joint in [N.m].            %
% 	ubar(:,2): torque applied to link 3 at shoulder joint in [N.m].       %    
%   ubar(:,3): horizontal force applied at shoulder joint in [N].         %
%   ubar(:,4): vertical force applied at shoulder joint in [N].           %
% Ktvmat: LTV toolbox object with 4 by 6 time-varying LQR gain.           %
% p: parameters of the system.                                            %
% 	p(1): Mass of link 1 in [kg].                                         %
%   p(2): Mass of link 2 in [kg].                                         %
%   p(3): Mass of link 3 in [kg].                                         %
%   p(4): Moment of inertia of link 1 about its Center of Mass (CoM)      %
%         in [kg.m^2].                                                    %
%   p(5): Moment of inertia of link 2 about its CoM in [kg.m^2].          %
%   p(6): Moment of inertia of link 3 about its CoM in [kg.m^2].          %
%   p(7): Length of link 1 in [m].                                        %
%   p(8): Length of link 2 in [m].                                        %
%   p(9): Length of link 3 in [m].                                        %
%   p(10): Distance from ankle joint to CoM of link 1 in [m].             %
%   p(11): Distance from knee joint to CoM of link 2 in [m].              %
%   p(12): Distance from hip joint to CoM of link 3 in [m].               %
% epsilon: initial spacing of the two points in the finite difference     %
%   formula used for linearization of the three-link robot dynamics.      %
% err: error tolerance in the linearization of the three-link robot       %
%   dynamics.                                                             %
% maxiter: maximum number of iterations allowed for computing the         %
%   linearization of the three-link robot dynamics.                       %
%                                                                         %
% Output                                                                  %
%                                                                         %
% xdot: time derivative of the state of the three-link robot, and the     %
%       entries of the sensitivity matrix.                                %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdot = TVLQRSTS3LinkSensitivityPvec(t,x,tgrid,xbar,ubar,Ktvmat,p,epsilon,err,maxiter)

% Parameters of the system
m1 = p(1);     % Mass of link 1 [kg]. 
m2 = p(2);     % Mass of link 2 [kg].
m3 = p(3);     % Mass of link 3 [kg]. 
I1 = p(4);     % Moment of inertia of link 1 about its CoM.
I2 = p(5);     % Moment of inertia of link 2 about its CoM.
I3 = p(6);     % Moment of inertia of link 3 about its CoM.
l1 = p(7);     % Length of link 1 [m].
l2 = p(8);     % Length of link 2 [m].
l3 = p(9);     % Length of link 3 [m].
lc1 = p(10);   % Distance from ankle joint to CoM of link 1 [m].
lc2 = p(11);   % Distance from knee joint to CoM of link 2 [m].
lc3 = p(12);   % Distance from hip joint to CoM of link 3 [m].
g = 9.81;      % Acceleration of gravity [m/s^2].

% Vector of system parameters. 
pbar = [m1;m2;m3;I1;I2;I3;l1;l2;l3;lc1;lc2;lc3];

% Number of states for the three-link planar robot.
nx = size(xbar,2);

% Number of external inputs applied to the three-link planar robot. 
nu = size(ubar,2);

% Number of parameters for the robot. 
np = (nx/2)*4;

% Reference state at time t from cubic spline interpolation.
xr = zeros(nx,1);
for i=1:nx
    xr(i) = csapi(tgrid,xbar(:,i),t);
end

% Reference input at time t from cubic spline interpolation.
ur = zeros(nu,1);
for i=1:nu
    ur(i) = csapi(tgrid,ubar(:,i),t);
end

% LQR gain.
K = eval2obj(Ktvmat,t);

% State deviation.  
dx = x(1:nx)-xr;

% Input from TV LQR.
du = -K.MatArray*dx;

% Number of control inputs. 
nc = numel(du);

% System input
if nc == 1
    u = ur + [du;0;0;0];
elseif nc == nu
    u = ur + du;
else
    error('Mismatch in dimensions of LQR gain and input.')
end

% State of three-link planar robot.
th = x(1:3);
om = x(4:6);

% Terms with trigonometric functions. 
s2 = sin(th(2));
s3 = sin(th(3));
s23 = sin(th(2)+th(3));
c2 = cos(th(2));
c3 = cos(th(3));
c23 = cos(th(2)+th(3));
sv = sin([th(1);th(1)+th(2);th(1)+th(2)+th(3)]);
cv = cos([th(1);th(1)+th(2);th(1)+th(2)+th(3)]);

% Mass matrix.
M11 = I1 + I2 + I3 + lc1^2*m1 + m2*(l1^2 + 2*l1*lc2*c2 + lc2^2) + m3*(l1^2 + 2*l1*l2*c2 + 2*l1*lc3*c23 + l2^2 + 2*l2*lc3*c3 + lc3^2);
M12 = I2 + I3 + lc2*m2*(l1*c2 + lc2) + m3*(l1*l2*c2 + l1*lc3*c23 + l2^2 + 2*l2*lc3*c3 + lc3^2);
M13 = I3 + lc3*m3*(l1*c23 + l2*c3 + lc3);
M22 = I2 + I3 + lc2^2*m2 + m3*(l2^2 + 2*l2*lc3*c3 + lc3^2);
M23 = I3 + lc3*m3*(l2*c3 + lc3);
M33 = I3 + lc3^2*m3;
M = [M11,M12,M13;M12,M22,M23;M13,M23,M33];
Minv = M\eye(3);

% Constant terms.
% k0 = 1/(m1+m2+m3); % Only used for inverse kinematics of the CoM. 
k1 = lc1*m1+l1*m2+l1*m3;
k2 = lc2*m2+l2*m3;
k3 = lc3*m3;
k4 = l1*(k2*s2+k3*s23);
k5 = k3*l2*s3;

% Vector of energy contributions due to gravity effects.
fg = -g*[k1,k2,k3;0,k2,k3;0,0,k3]*cv;

% Coriolis effects Matrix.
C = [k4,-k2*l1*s2+k3*l2*s3,-k3*l1*s23-k3*l2*s3;k4,k5,-k5;l1*k3*s23,k5,0];

% Squares of angular velocities of the links in the inertial frame.  
omsq = [om(1);om(1)+om(2);om(1)+om(2)+om(3)].^2;

% Energy contributions due to gravity, and Coriolis effects.
f = fg-C*omsq;

% Matrix of generalized force.
Atau = [[0;0;1],-ones(3,1),-[l1,l2,l3;0,l2,l3;0 0 l3]*sv,[l1,l2,l3;0,l2,l3;0 0 l3]*cv];

% Preallocate array for time derivatives. 
xdot = zeros(nx+nx*np,1);

% Time derivatives of the state of the three-link robot.
xdot(1:3) = om;
xdot(4:6) = Minv*(f+Atau*u);

% Structure with invariant parameters of the three-link robot.
par.g = g;       % Acceleration of gravity [m/s^2].

% Linearization of three-link robot dynamics at time t.
[A, B1, B2] = ParameterLinearization(@STSThreeLinkPar,xr',pbar',ur',par,epsilon,err,maxiter);

% Affine function describing the change in time of the sensitivity matrix.
Sdot = (A-B2*K.MatArray)*reshape(x(nx+1:end),[np,nx])' + B1;

% Time derivative of the sensitivity matrix.
xdot(nx+1:end) = vec(Sdot');