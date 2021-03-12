%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Plot upper and lower bounds for the state, input, and center of mass    %
% (CoM) trajectories for n numerical simulations of a three-link robot    %
% system.                                                                 %
%                                                                         %
% Input                                                                   %
%                                                                         %
% T: 1 by n cell array with time samples from simulations.                %
% 	T{i}: m by 1 time array.                                              %
% X: 1 by n cell array with state data.                                   %
% 	X{i}: m by 6 state array.                                             %
% U: 1 by n cell array with input data.                                   %
% 	U{i}: m by nu input array.                                            %
% p: n by 12 array of parameters of the three-link robot.                 %
%   p(:,1): Mass of link 1 [kg].                                          %
%   p(:,2): Mass of link 2 [kg].                                          %
%   p(:,3): Mass of link 3 [kg].                                          %
%   p(:,4): Moment of inertia of link 1 about its CoM.                    %
%   p(:,5): Moment of inertia of link 2 about its CoM.                    %
%   p(:,6): Moment of inertia of link 3 about its CoM.                    %
%   p(:,7): Length of link 1 [m].                                         %
%   p(:,8): Length of link 2 [m].                                         %
%   p(:,9): Length of link 3 [m].                                         %
%   p(:,10): Distance from ankle joint to CoM of link 1 [m].              % 
%   p(:,11): Distance from knee joint to CoM of link 2 [m].               %
%   p(:,12): Distance from hip joint to CoM of link 3 [m].                %
% tbnd: ns by 1 time array for over-approximation boxes.                  %
% Xlbnd: ns by 6 array with lower bounds for the state.                   %
% Xubnd: ns by 6 array with upper bounds for the state.                   %
% Ulbnd: ns by nu array with lower bounds for the input.                  %
% Uubnd: ns by nu array with upper bounds for the input.                  %
% Zlbnd: ns by 6 array with lower bounds for the CoM.                     %
% Zubnd: ns by 6 array with upper bounds for the CoM.                     %
% config: Integer defining the configuration of the input u.              %
% 	Torques are in [N.m].                                                 %
% 	The forces are applied at the shoulder joint, and are in [N].         %
% 	1: u=[ankle torque; shoulder torque; Fx; Fy].                         %
% 	2: u=[knee torque; shoulder torque; Fx; Fy].                          %
% 	3: u=[hip torque; shoulder torque; Fx; Fy].                           %
% 	4: u=[ankle torque; knee torque; shoulder torque; Fx; Fy].            %
% 	5: u=[ankle torque; hip torque; shoulder torque; Fx; Fy].             %
% 	6: u=[knee torque; hip torque; shoulder torque; Fx; Fy].              %
% 	7: u=[ankle torque; knee torque; hip torque; shoulder torque; Fx; Fy].%
% 	8: u=[ankle torque; knee torque; hip torque; shoulder torque].        %
% 	9: u=[ankle torque; knee torque; hip torque].                         %
% ind: Boolean variable for controlling the display of the plots.         %
%   If True, all plots are presented in independent figures.              %
%   If False, plots are grouped in figures for the state, input, and      %
%   output trajectories.                                                  %
% txt1: string of characters for legend 1.                                %
% txt2: string of characters for legend 2.                                %
% txt3: string of characters for legend 3.                                %
% txt4: string of characters for legend 4.                                %
% txt5: string of characters for legend 5.                                %
%                                                                         %
% Output                                                                  %
%                                                                         %
% Show plots on screen.                                                   %
% Z: 1 by n cell array with output data.                                  %
%   Z{i}: 6 by m output array.                                            %
%     Z{i}(1,:): angular position of link 2 relative to link 1 in [rad].  %
%     Z{i}(2,:): x coordinate of the CoM position in [m].                 %
%     Z{i}(3,:): y coordinate of the CoM position in [m].                 %
%     Z{i}(4,:): angular velocity of link 2 in [rad/s].                   %
%     Z{i}(5,:): x coordinate of the CoM velocity in [m/s].               %
%     Z{i}(6,:): y coordinate of the CoM velocity in [m/s].               %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Z = plot3LinkBoundsNew(T,X,U,p,tbnd,Xlbnd,Xubnd,Ulbnd,Uubnd,Zlbnd,Zubnd,config,ind,txt1,txt2,txt3,txt4,txt5)

% Number of simulations.
ne = numel(T);

% Numer of states.
nx = size(X{1},2);

% Initialize figures.
if not(ind)
    figure()
end
for i=1:nx
    if ind
        figure(i)
    else
        subplot(2,3,i)
    end
    hold on
    plot(T{end},X{end}(:,i)*180/pi,'b-',T{end},X{end}(:,i)*180/pi,'r-','LineWidth',2)
    plot(T{end},X{end}(:,i)*180/pi,'color','[0,0.5,0]','LineWidth',2)
    legend({txt1,txt2,txt3},'Interpreter','Latex','Location','Best')
end

% Plot state trajectories with uncertainties in the parameters.
for j = 1:ne-1
    for i=1:nx
        if ind
            figure(i)
        else
            subplot(2,3,i)
        end
        plot(T{j},X{j}(:,i)*180/pi,'b-','LineWidth',2)
    end
end

% Plot reference state trajectories and bounds.
for i=1:nx
    if ind
        figure(i)
    else
        subplot(2,3,i)
    end
    plot(T{end},X{end}(:,i)*180/pi,'r-','LineWidth',2)
    plot(tbnd,Xlbnd(:,i)*180/pi,tbnd,Xubnd(:,i)*180/pi,'color','[0,0.5,0]','LineWidth',2)
    xlim([T{end}(1) T{end}(end)])
    grid
	% Add labels.
    xlabel('$t\:[s]$','Interpreter','Latex')
    if i <= (nx/2)
        lbl = ['$\theta_{' num2str(i) '}(t)\:[^\circ]$'];
        ylabel(lbl,'Interpreter','Latex')
    else
        lbl = ['$\dot{\theta}_{' num2str(i-nx/2) '}(t)\:[^\circ/s]$'];
        ylabel(lbl,'Interpreter','Latex')
    end
end

% Number of inputs
nu = size(U{1},2);

% Plot input trajectories depending on configuration.
if not(ind)
    figure()
end
switch config
    case 1
        % Upper body loads and actuation at the ankle.
        % u = [ankle torque, shoulder torque, Fx, Fy]'
        % Add legends to input plots
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,2,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories, and input bounds.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
        
    case 2
        % Upper body loads and actuation at the knee.
        % u = [knee torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,2,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories, and input bounds.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 3
        % Upper body loads and actuation at the hip.
        % u = [hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,2,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories, and input bounds.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels.
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 4
        % Upper body loads, and actuation at the ankle and knee.
        % u = [ankle torque, knee torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,3,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==4
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 5
        % Upper body loads and actuation at the ankle and hip.
        % u = [ankle torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt3},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,3,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==4
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 6
        % Upper body loads and actuation at the knee and hip.
        % u = [knee torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,3,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==4
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 7
        % Upper body loads and actuation at the ankle, knee and hip.
        % u = [ankle torque, knee torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,3,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,3,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==4
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==5
                ylabel('$F_x(t)\:[N]$','Interpreter','Latex');
            else
                ylabel('$F_y(t)\:[N]$','Interpreter','Latex');
            end
        end
    case 8
        % Actuation at the ankle, knee, hip and shoulder.
        % u = [ankle torque, knee torque, hip torque, shoulder torque]'
        
        % Input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(2,2,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(2,2,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==3
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            else
                ylabel('$\tau_{s}(t)\:[N.m]$','Interpreter','Latex');
            end
        end
    case 9
        % Actuation at the ankle, knee and hip.
        % u = [ankle torque, knee torque, hip torque]'
        
        % Input plots.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(1,3,i)
            end
            hold on
            plot(T{end},U{end}(:,i),'b-',T{end},U{end}(:,i),'r-','LineWidth',2);
            plot(T{end},U{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories with parameter uncertainties.
        for j = 1:ne-1
            for i=1:nu
                if ind
                    figure(nx+i)
                else
                    subplot(1,3,i)
                end
                plot(T{j},U{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectories.
        for i=1:nu
            if ind
                figure(nx+i)
            else
                subplot(1,3,i)
            end
            plot(T{end},U{end}(:,i),'r-','LineWidth',2)
            plot(tbnd,Ulbnd(:,i),tbnd,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2)
            xlim([T{end}(1) T{end}(end)])
            grid
			% Add labels. 
            xlabel('$t\:[s]$','Interpreter','Latex')
            if i==1
                ylabel('$\tau_{a}(t)\:[N.m]$','Interpreter','Latex');
            elseif i==2
                ylabel('$\tau_{k}(t)\:[N.m]$','Interpreter','Latex');
            else
                ylabel('$\tau_{h}(t)\:[N.m]$','Interpreter','Latex');
            end
        end
    otherwise
        error('Unrecognized configuration of inputs. Options range from 1 to 9.')
end

% Cell for Center of Mass trajectories
Z = cell(1,ne);

% Compute Center of Mass Trajectories
for j=1:ne
    Z{j} = xpar2CoMpv(X{j}',p(j,:));
end

% Add legends to plots.
if not(ind)
    figure()
end

% Plot x coordinate of the CoM position.
if ind
    figure(nx+nu+1)
else
    subplot(2,3,1)
end
hold on
plot(T{end},Z{end}(2,:),'b-',T{end},Z{end}(2,:),'r-','LineWidth',2);
plot(T{end},Z{end}(2,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot y coordinate of the CoM position.
if ind
    figure(nx+nu+2)
else
    subplot(2,3,2)
end
hold on
plot(T{end},Z{end}(3,:),'b-',T{end},Z{end}(3,:),'r-','LineWidth',2);
plot(T{end},Z{end}(3,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot position of the CoM on the sagittal plane. 
if ind
    figure(nx+nu+3)
else
    subplot(2,3,3)
end
hold on
plot(Z{end}(2,:),Z{end}(3,:),'b-',Z{end}(2,:),Z{end}(3,:),'r-','LineWidth',2);
plot(Z{end}(2,:),Z{end}(3,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot x coordinate of the CoM velocity. 
if ind
    figure(nx+nu+4)
else
    subplot(2,3,4)
end
hold on
plot(T{end},Z{end}(5,:),'b-',T{end},Z{end}(5,:),'r-','LineWidth',2);
plot(T{end},Z{end}(5,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot y coordinate of the CoM velocity. 
if ind
    figure(nx+nu+5)
else
    subplot(2,3,5)
end
hold on
plot(T{end},Z{end}(6,:),'b-',T{end},Z{end}(6,:),'r-','LineWidth',2);
plot(T{end},Z{end}(6,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot velocity of the CoM. 
if ind
    figure(nx+nu+6)
else
    subplot(2,3,6)
end
hold on
plot(Z{end}(5,:),Z{end}(6,:),'b-',Z{end}(5,:),Z{end}(6,:),'r-','LineWidth',2);
plot(Z{end}(5,:),Z{end}(6,:),'color','[0,0.5,0]','LineWidth',2);
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot CoM trajectories.
for i = 1:nx
    if ind
        figure(nx+nu+i)
    else
        subplot(2,3,i)
    end
    for j = 1:ne-1
        if i==1
            % xCoM(t)
            plot(T{j},Z{j}(2,:),'b-','LineWidth',2);
        elseif i==2
            % yCoM(t)
            plot(T{j},Z{j}(3,:),'b-','LineWidth',2);
        elseif i==3
            % xCoM(t) vs. yCoM(t)
            plot(Z{j}(2,:),Z{j}(3,:),'b-','LineWidth',2);
        elseif i==4
            % vxCoM(t)
            plot(T{j},Z{j}(5,:),'b-','LineWidth',2);
        elseif i==5
            % vyCoM(t)
            plot(T{j},Z{j}(6,:),'b-','LineWidth',2);
        else
            % vxCoM(t) vs. vyCoM(t)
            plot(Z{j}(5,:),Z{j}(6,:),'b-','LineWidth',2);
        end
    end
end

% Plot reference CoM trajectories.
for i = 1:nx
    if ind
        figure(nx+nu+i)
    else
        subplot(2,3,i)
    end
    if i==1
        % x coordinate of the CoM position. 
        plot(T{end},Z{end}(2,:),'r-','LineWidth',2);
        plot(tbnd,Zlbnd(:,2),tbnd,Zubnd(:,2),'color','[0,0.5,0]','LineWidth',2);
        xlim([T{end}(1),T{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$x_{CoM}\left(t\right)\:\left[m\right]$','Interpreter','Latex')
    elseif i==2
        % y coordinate of the CoM position. 
        plot(T{end},Z{end}(3,:),'r-','LineWidth',2);
        plot(tbnd,Zlbnd(:,3),tbnd,Zubnd(:,3),'color','[0,0.5,0]','LineWidth',2);
        xlim([T{end}(1),T{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$y_{CoM}\left(t\right)\:\left[m\right]$','Interpreter','Latex')
    elseif i==3
        % Position of the CoM. 
        plot(Z{end}(2,:),Z{end}(3,:),'r-','LineWidth',2);
        plot(Zlbnd(:,2),Zlbnd(:,3),Zubnd(:,2),Zubnd(:,3),[Zlbnd(1,2),Zubnd(1,2),Zubnd(1,2)],[Zlbnd(1,3),Zlbnd(1,3),Zubnd(1,3)],[Zlbnd(end,2),Zlbnd(end,2),Zubnd(end,2)],[Zlbnd(end,3),Zubnd(end,3),Zubnd(end,3)],'color','[0,0.5,0]','LineWidth',2);
        ylim([0.6,1.05]);
        xlabel('$x_{CoM}\:\left[m\right]$','Interpreter','Latex')
        ylabel('$y_{CoM}\:\left[m\right]$','Interpreter','Latex')
        axis equal
    elseif i==4
        % x coordinate of the CoM velocity.
        plot(T{end},Z{end}(5,:),'r-','LineWidth',2);
        plot(tbnd,Zlbnd(:,5),tbnd,Zubnd(:,5),'color','[0,0.5,0]','LineWidth',2);
        xlim([T{end}(1),T{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$\dot{x}_{CoM}\left(t\right)\:\left[m/s\right]$','Interpreter','Latex')
    elseif i==5
        % y coordinate of the CoM velocity. 
        plot(T{end},Z{end}(6,:),'r-','LineWidth',2);
        plot(tbnd,Zlbnd(:,6),tbnd,Zubnd(:,6),'color','[0,0.5,0]','LineWidth',2);
        xlim([T{end}(1),T{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$\dot{y}_{CoM}\left(t\right)\:\left[m/s\right]$','Interpreter','Latex')
    else
        % Velocity of the CoM. 
        [~,idx] = max(Zubnd(:,6));
        plot(Z{end}(5,:),Z{end}(6,:),'r-','LineWidth',2);
        plot(Zlbnd(:,5),Zlbnd(:,6),Zubnd(:,5),Zubnd(:,6),[Zlbnd(idx,5),Zlbnd(idx,5),Zubnd(idx,5)],[Zlbnd(idx,6),Zubnd(idx,6),Zubnd(idx,6)],[Zlbnd(end,5),Zubnd(end,5),Zubnd(end,5)],[Zlbnd(end,6),Zlbnd(end,6),Zubnd(end,6)],'color','[0,0.5,0]','LineWidth',2);
        ylim([-0.02,0.16]);
        xlabel('$\dot{x}_{CoM}\:\left[m/s\right]$','Interpreter','Latex')
        ylabel('$\dot{y}_{CoM}\:\left[m/s\right]$','Interpreter','Latex')
        axis equal
    end
    grid()
end