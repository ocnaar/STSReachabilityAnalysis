%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Jacobian linearization of a time-invariant function f about state,      %
% input, and parameter trajectories using two-point finite difference     %
% formulas.                                                               %
%                                                                         %
% Input                                                                   %
% 	f: function handle.                                                   %
% 	xbar: T by nx array with state trajectory.                            %
% 	pbar: T by np array with parameter trajectory.                        %
% 	ubar: T by nu array with input trajectory.                            %
% 	par: invariant parameters of function f.                              %
% 	eps: initial spacing for the two points in finite difference formula. %
% 	err: error tolerance in succesive computations of matrix columns.     %
% 	maxiter: maximum number of iterations allowed for computing matrix    %
% 		columns.                                                          %
%                                                                         %
% Output                                                                  %
% 	A: nx by nx by T array of system matrices.                            %
% 	B1: nx by np by T array of parameter disturbance matrices.            %
% 	B2: nx by nu by T array of input matrices.                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B1, B2] = ParameterLinearization(f,xbar,pbar,ubar,par,eps,err,maxiter)

% Number of sampled points from state, input, and parameter trajectories.
% Number of states.
[T,nx] = size(xbar);

% Number of parameters.
np = size(pbar,2);

% Number of inputs.
nu = size(ubar,2);

% Array for storing system matrices A.
A = zeros(nx,nx,T);

% Array for storing parameter disturbance matrices B1.
B1 = zeros(nx,np,T);

% Array for storing input matrices B2.
B2 = zeros(nx,nu,T);

% Perform linearization about sampled points. 
for k = 1:T
    
    % Value of function along state, parameter, and input trajectories at
    % time k. 
    f0 = f(0,xbar(k,:)',pbar(k,:)',ubar(k,:)',par);
    
    % Compute system matrix A at time k.
    I = eye(nx);
    for i=1:nx
        j=1;
        a = ones(nx,1);
        aprev = zeros(nx,1);
        while norm(a-aprev)>err && j<=maxiter
            delta = eps/j;
            aprev = a;
            f1 = f(0,xbar(k,:)'+delta*I(:,i),pbar(k,:)',ubar(k,:)',par);
            a=(f1-f0)/delta;
            j = j+1;
        end
        A(:,i,k) = a;
    end;
    
    % Compute matrix B1 at time k.
    I = eye(np);
    for i=1:np
        j=1;
        b1 = ones(nx,1);
        bprev = zeros(nx,1);
        while norm(b1-bprev)>err && j<=maxiter
            delta = eps/j;
            bprev = b1;
            f1 = f(0,xbar(k,:)',pbar(k,:)'+delta*I(:,i),ubar(k,:)',par);
            b1=(f1-f0)/delta;
            j = j+1;
        end
        B1(:,i,k) = b1;
    end
    
    % Compute matrix B2 at time k.
    I = eye(nu);
    for i=1:nu
        j=1;
        b2 = ones(nx,1);
        bprev = zeros(nx,1);
        while norm(b2-bprev)>err && j<=maxiter
            delta = eps/j;
            bprev = b2;
            f1 = f(0,xbar(k,:)',pbar(k,:)',ubar(k,:)'+delta*I(:,i),par);
            b2=(f1-f0)/delta;
            j = j+1;
        end
        B2(:,i,k) = b2;
    end
end