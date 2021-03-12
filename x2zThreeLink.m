%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% This function maps the state of a three-link planar robot with revolute %
% joints into the position, and velocity of its center of mass (CoM). It  %
% also computes the Jacobian matrices of the mapping relative to the      %
% state, and the parameter.                                               %
%                                                                         %
% Input                                                                   %
%                                                                         %
% x: 6 by nt array of state trajectory.                                   %
% 	x(1,:): angular position of link 1 relative to the horizontal [rad].  %
% 	x(2,:): angular position of link 2 relative to link 1 in [rad].       %
% 	x(3,:): angular position of link 3 relative to link 2 in [rad].       %
% 	x(4,:): angular velocity of link 1 in [rad/s].                        %
% 	x(5,:): angular velocity of link 2 in [rad/s].                        %
% 	x(6,:): angular velocity of link 3 in [rad/s].                        %
% p: 12 by 1 array of the three-link robot parameters.                    %
%   p(1): Mass of link 1 in [kg].                                         %
%   p(2): Mass of link 2 in [kg].                                         %
%   p(3): Mass of link 3 in [kg].                                         %
%   p(4): Moment of inertia of link 1 about its CoM in [kg.m^2].          %
%   p(5): Moment of inertia of link 2 about its CoM in [kg.m^2].          %
%   p(6): Moment of inertia of link 3 about its CoM in [kg.m^2].          %
%   p(7): Length of link 1 in [m].                                        %
%   p(8): Length of link 2 in [m].                                        %
%   p(9): Length of link 3 in [m].                                        %
%   p(10): Distance from ankle joint to CoM of link 1 in [m].             %
%   p(11): Distance from knee joint to CoM of link 2 in [m].              %
%   p(12): Distance from hip joint to CoM of link 3 in [m].               %
%                                                                         %
% Output                                                                  %
%                                                                         %
% z: 6 by nt array of output trajectory.                                  %
%   z(1,:): angular position of link 2 relative to link 1 in [rad].       %
%   z(2,:): x coordinate of the CoM position in [m].                      %
%   z(3,:): y coordinate of the CoM position in [m].                      %
%   z(4,:): angular velocity of link 2 [rad/s].                           %
%   z(5,:): x coordinate of the velocity of the CoM in [m/s].             %
%   z(6,:): y coordinate of the velocity of the CoM in [m/s].             %
% Jx: 6 by 6 by nt array with Jacobian matrices about state trajectory.   %
% Jp: 6 by 12 by nt array with Jacobian matrices about the parameters of  %
%     the system.                                                         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, Jx, Jp] = x2zThreeLink(x,p)

% Parameters of the system
m1 = p(1);     % Mass of link 1 [kg].
m2 = p(2);     % Mass of link 2 [kg].
m3 = p(3);     % Mass of link 3 [kg].
l1 = p(7);     % Length of link 1 [m].
l2 = p(8);     % Length of link 2 [m].
lc1 = p(10);   % Distance from ankle joint to CoM of link 1 [m].
lc2 = p(11);   % Distance from knee joint to CoM of link 2 [m].
lc3 = p(12);   % Distance from hip joint to CoM of link 3 [m].

% Constant terms.
k0 = 1/(m1+m2+m3);
k1 = lc1*m1+l1*m2+l1*m3;
k2 = lc2*m2+l2*m3;
k3 = lc3*m3;

% Number of states. 
nx = size(x,1);

% Number of time steps. 
nt = size(x,2);

% Number of parameters. 
np = numel(p);

% Array for mapping vectors in the theta space to the z space.
z = zeros(nx,nt);

% Arrays for storing Jacobian matrices.
Jx = zeros(nx,nx,nt);  % About states.
Jp = zeros(nx,np,nt);  % About parameters.

for i=1:nt
    % Angular positions of the links.
    th = x(1:3,i);
    
    % Sine and cosine functions
    s1 = sin(th(1));
    s12 = sin(th(1)+th(2));
    s123 = sin(th(1)+th(2)+th(3));
    c1 = cos(th(1));
    c12 = cos(th(1)+th(2));
    c123 = cos(th(1)+th(2)+th(3));
    
    % Terms involving sine and cosine functions. 
    k4 = k0*(k1*s1+k2*s12+k3*s123);
    k5 = k0*(k2*s12+k3*s123);
    k6 = k0*k3*s123;
    k7 = k0*(k1*c1+k2*c12+k3*c123);
    k8 = k0*(k2*c12+k3*c123);
    k9 = k0*k3*c123;
    
    % Angular position of the knee.
    z(1,i) = x(2,i);
    
    % Calculate position coordinates of the CoM.
    z(2,i) = k7;    % x coordinate.
    z(3,i) = k4;    % y coordinate.
    
    % Angular velocities of the links.
    om = x(4:6,i);
    
    % Angular velocity of the knee.
    z(4,i) = om(2);
    
    % Calculate velocity of the CoM.
    z(5,i) = -om(1)*k4-om(2)*k5-om(3)*k6; % x coordinate.
    z(6,i) = om(1)*k7+om(2)*k8+om(3)*k9;  % y coordinate.
    
    % Evaluate Jacobian about state trajectories.
    Jx(:,:,i) = [0, 1, 0, 0, 0, 0; -k4, -k5, -k6, 0, 0, 0; k7, k8, k9, 0, 0, 0;... 
    0, 0, 0, 0, 1, 0; -k7*om(1)-k8*om(2)-k9*om(3), -k8*(om(1)+om(2))-k9*om(3),... 
    -k9*(om(1)+om(2)+om(3)),-k4,-k5,-k6; -k4*om(1)-k5*om(2)-k6*om(3),...
    -k5*(om(1)+om(2))-k6*om(3), -k6*(om(1)+om(2)+om(3)), k7, k8, k9];

    % Evaluate Jacobian about parameters.
    Jp(:,:,i) = [zeros(1,np);... 
        k0*(lc1*c1-k7), k0*(l1*c1+lc2*c12-k7), k0*(l1*c1+l2*c12+lc3*c123-k7),...
        0, 0, 0, k0*(m2+m3)*c1, k0*m3*c12, 0, k0*m1*c1, k0*m2*c12, k0*m3*c123;...
        k0*(lc1*s1-k4), k0*(l1*s1+lc2*s12-k4), k0*(l1*s1+l2*s12+lc3*s123-k4),...
        0, 0, 0, k0*(m2+m3)*s1, k0*m3*s12, 0, k0*m1*s1, k0*m2*s12, k0*m3*s123;...
        zeros(1,np);...
        -om(1)*k0*(lc1*s1-k4)+om(2)*(k0^2)*(k2*s12+k3*s123)+om(3)*(k0^2)*lc3*m3*s123,...
        -om(1)*k0*(l1*s1+lc2*s12-k4)+om(2)*(k0^2)*(k2*s12+k3*s123)-om(2)*k0*lc2*s12+om(3)*(k0^2)*lc3*m3*s123,...
        -om(1)*k0*(l1*s1+l2*s12+lc3*s123-k4)+om(2)*(k0^2)*(k2*s12+k3*s123)-om(2)*k0*(l2*s12+lc3*s123)+om(3)*(k0^2)*lc3*m3*s123-om(3)*k0*lc3*s123,...
        0, 0, 0, -om(1)*k0*(m2+m3)*s1, -k0*m3*s12*(om(1)+om(2)), 0,...
        -om(1)*k0*m1*s1, -k0*m2*s12*(om(1)+om(2)), -k0*m3*s123*sum(om);...
        om(1)*k0*(lc1*c1-k7)-om(2)*(k0^2)*(k2*c12+k3*c123)-om(3)*(k0^2)*lc3*m3*c123,...
        om(1)*k0*(l1*c1+lc2*c12-k7)-om(2)*(k0^2)*(k2*c12+k3*c123)+om(2)*k0*lc2*c12-om(3)*(k0^2)*lc3*m3*c123,...
        om(1)*k0*(l1*c1+l2*c12+lc3*c123-k7)-om(2)*(k0^2)*(k2*c12+k3*c123)+om(2)*k0*(l2*c12+lc3*c123)-om(3)*(k0^2)*lc3*m3*c123+om(3)*k0*lc3*c123,...
        0, 0, 0, om(1)*k0*(m2+m3)*c1, k0*m3*c12*(om(1)+om(2)), 0,...
        om(1)*k0*m1*c1, k0*m2*c12*(om(1)+om(2)), k0*m3*c123*sum(om)];
end