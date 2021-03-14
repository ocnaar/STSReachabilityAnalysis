%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez-Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Events function for simulation of the three-link robot under finite time%
% horizon LQR control.                                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [event,isterminal,direction] = TVLQRSTS3LinkEvents(t,x,xmin,xmax)

% Number of states.
nx = numel(xmin);

% "Zero" events.
event(1:nx) = x(1:nx)-xmin;
event(nx+1:2*nx) = xmax-x(1:nx); 

% Halt integration when events are triggered.
isterminal(1:2*nx) = 1;

% The zero can be approached from above (event is decreasing to 0).
direction = -1;