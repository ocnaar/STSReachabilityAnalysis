%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Summer 2017                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Plot state, input, and output portraits with over-approximation boxes   %
% for the closed-loop dynamics of a three-link planar robot.              %
%                                                                         %
% Input                                                                   %
%                                                                         %
% tspan: m by 1 time array for over-approximation boxes.                  %
% Xbounds: m by 6 by 2 array.                                             %
% 	Xbounds(:,:,1): state lower bounds.                                   %
% 	Xbounds(:,:,2): state upper bounds.                                   %
% Ubounds: m by nu by 2 array.                                            %
% 	Ubounds(:,:,1): input lower bounds.                                   %
% 	Ubounds(:,:,2): input upper bounds.                                   %
% Zbounds: m by 6 by 2 array.                                             %
% 	Zbounds(:,:,1): output lower bounds.                                  %
% 	Zbounds(:,:,2): output upper bounds.                                  %
% Tcell: 1 by n cell array with time samples from simulations.            %
% 	T{i}: ns by 1 time array.                                             %
% Xcell: 1 by n cell array with state data from simulations.              %
% 	X{i}: ns by 6 state array.                                            %
% Ucell: 1 by n cell array with input data from simulations.              %
% 	U{i}: ns by nu input array.                                           %
% pexp: n-1 by 12 array of parameters sampled within a known interval.    %
%   p(:,1): Mass of link 1 [kg].                                          %
%   p(:,2): Mass of link 2 [kg].                                          %
%   p(:,3): Mass of link 3 [kg].                                          %
%   p(:,4): Moment of inertia of link 1 about its Center of Mass (CoM).   %
%   p(:,5): Moment of inertia of link 2 about its CoM.                    %
%   p(:,6): Moment of inertia of link 3 about its CoM.                    %
%   p(:,7): Length of link 1 [m].                                         %
%   p(:,8): Length of link 2 [m].                                         %
%   p(:,9): Length of link 3 [m].                                         %
%   p(:,10): Distance from ankle joint to CoM of link 1 [m].              % 
%   p(:,11): Distance from knee joint to CoM of link 2 [m].               %
%   p(:,12): Distance from hip joint to CoM of link 3 [m].                %
% pnom: 1 by 12 array with nominal values for the parameter of the system.%
%   p(1): Mass of link 1 [kg].                                            %
%   p(2): Mass of link 2 [kg].                                            %
%   p(3): Mass of link 3 [kg].                                            %
%   p(4): Moment of inertia of link 1 about its CoM.                      %
%   p(5): Moment of inertia of link 2 about its CoM.                      %
%   p(6): Moment of inertia of link 3 about its CoM.                      %
%   p(7): Length of link 1 [m].                                           %
%   p(8): Length of link 2 [m].                                           %
%   p(9): Length of link 3 [m].                                           %
%   p(10): Distance from ankle joint to CoM of link 1 [m].                %
%   p(11): Distance from knee joint to CoM of link 2 [m].                 %
%   p(12): Distance from hip joint to CoM of link 3 [m].                  %
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
% store: Boolean variable for saving plots in the current directory.      %
% plat: string of characters with type of computer in use. Available      %
%		options are {'PC','MAC'}.                                         %
%                                                                         %
% Output                                                                  %
%                                                                         %
% Show plots on screen, and save image copies in working directory.       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function PlotOverAppBoxes(tspan,Xbounds,Ubounds,Zbounds,Tcell,Xcell,Ucell,pexp,pnom,config,store,plat)

% Settings for exporting Matlab figures to images. 
if store
    % Get export setup from style file and define format.
    sname = 'default';
    s = hgexport('readstyle',sname);
    s.Format = 'png';
    dirname = strrep(datestr(datetime), ':', '_');
    if strcmp(plat,'PC')
        dirname = ['PerformanceTests\', dirname];
    elseif strcmp(plat,'MAC')
        dirname = ['PerformanceTests/', dirname];
    else
        error('No supported platform.');
    end
    mkdir(dirname);
end

% Reference trajectories for states. 
xbar = Xcell{end};

% Reference trajectories for inputs. 
ubar = Ucell{end};

% Number of states. 
nx = size(xbar,2);

% Number of inputs. 
nu = size(ubar,2);

% Number of sampled parameter values for simulation. 
nexp = size(pexp,1);

% Number of sampled times where over-approximations were computed.
nover = numel(tspan);

% Texts for legends in variables over time plots. 
txt1 = 'Trajectories for $p \in \mathcal{P}_s$'; 
txt2 = 'Reference Trajectory'; 

% Add legends to state plots.
for i=1:nx
    figure(i)
    hold on
    plot(Tcell{end},Xcell{end}(:,i)*180/pi,'b-',Tcell{end},Xcell{end}(:,i)*180/pi,'r-','LineWidth',2)
    plot(Tcell{end},Xcell{end}(:,i)*180/pi,'color','[0,0.5,0]','LineWidth',2)
    txt3 = ['$\underline{r_', num2str(i), '^x}(t) \: \& \: \overline{r_', num2str(i), '^x}(t) \: \mathrm{for} \: t \in T_s$'];
    legend({txt1,txt2,txt3},'Interpreter','Latex','Location','Best')
end

% Plot state trajectories for sampled parameters.
for j = 1:nexp
    for i=1:nx
        figure(i)
        plot(Tcell{j},Xcell{j}(:,i)*180/pi,'b-','LineWidth',2)
    end
end

% Lower and upper bounds for states. 
Xlbnd = Xbounds(:,:,1);
Xubnd = Xbounds(:,:,2);

% Plot reference state trajectory, and bounds.
for i=1:nx
    figure(i)
    plot(Tcell{end},Xcell{end}(:,i)*180/pi,'r-','LineWidth',2)
    plot(tspan,Xlbnd(:,i)*180/pi,tspan,Xubnd(:,i)*180/pi,'color','[0,0.5,0]','LineWidth',2)
    xlim([Tcell{end}(1) Tcell{end}(end)])
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
    % Export figure. 
    if store
        if strcmp(plat,'PC')
            fname = [dirname,'\x', num2str(i), '.png'];
        elseif strcmp(plat,'MAC')
            fname = [dirname,'/x', num2str(i), '.png'];
        else
            error('No supported platform.');
        end
        hgexport(gcf,fname,s);
    end
end

% Lower and upper bounds for inputs. 
Ulbnd = Ubounds(:,:,1);
Uubnd = Ubounds(:,:,2);

% Plot input trajectories depending on configuration.
switch config
    case 1
        % Upper body loads and actuation at the ankle.
        % u = [ankle torque, shoulder torque, Fx, Fy]'
        % Add legends to input plots
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory, and input bounds.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
			% Add labels.
            grid
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
        
    case 2
        % Upper body loads and actuation at the knee.
        % u = [knee torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory, and input bounds.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2)
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 3
        % Upper body loads and actuation at the hip.
        % u = [hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory, and input bounds.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 4
        % Upper body loads and actuation at the ankle, and knee.
        % u = [ankle torque, knee torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 5
        % Upper body loads and actuation at the ankle and hip.
        % u = [ankle torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 6
        % Upper body loads and actuation at the knee and hip.
        % u = [knee torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2)
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 7
        % Upper body loads and actuation at the ankle, knee and hip.
        % u = [ankle torque, knee torque, hip torque, shoulder torque, Fx, Fy]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 8
        % Actuation at the ankle, knee, hip, and shoulder.
        % u = [ankle torque, knee torque, hip torque, shoulder torque]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2)
            end
        end
        
        % Plot reference input trajectory, and bounds.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    case 9
        % Actuation at the ankle, knee, and hip.
        % u = [ankle torque, knee torque, hip torque]'
        
        % Add legends to input plots.
        for i=1:nu
            figure(nx+i)
            hold on
            plot(Tcell{end},Ucell{end}(:,i),'b-',Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(Tcell{end},Ucell{end}(:,i),'color','[0,0.5,0]','LineWidth',2);
            txt4 = ['$\underline{r_', num2str(i), '^u}(t) \: \& \: \overline{r_', num2str(i), '^u}(t)\: \mathrm{for} \: t \in T_s$'];
            legend({txt1,txt2,txt4},'Interpreter','Latex','Location','Best')
        end
        
        % Plot input trajectories for sampled parameters.
        for j = 1:nexp
            for i=1:nu
                figure(nx+i)
                plot(Tcell{j},Ucell{j}(:,i),'b-','LineWidth',2);
            end
        end
        
        % Plot reference input trajectory.
        for i=1:nu
            figure(nx+i)
            plot(Tcell{end},Ucell{end}(:,i),'r-','LineWidth',2);
            plot(tspan,Ulbnd(:,i),tspan,Uubnd(:,i),'color','[0,0.5,0]','LineWidth',2);
            xlim([Tcell{end}(1) Tcell{end}(end)])
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
            % Export figure.
            if store
                if strcmp(plat,'PC')
                    fname = [dirname,'\u', num2str(i), '.png'];
                elseif strcmp(plat,'MAC')
                    fname = [dirname,'/u', num2str(i), '.png'];
                else
                    error('No supported platform.');
                end
                hgexport(gcf,fname,s);
            end
        end
    otherwise
        error('Unrecognized configuration of inputs. Options range from 1 to 9.')
end

% Cell for Center of Mass trajectories.
Z = cell(1,nexp+1);

% Compute Center of Mass Trajectories for random parameters.
for j=1:nexp
    Z{j} = xpar2CoMpv(Xcell{j}',pexp(j,:));
end

% Compute CoM trajectory for the nominal value of the parameter.
Z{end} = xpar2CoMpv(Xcell{end}',pnom);

% Add legends to plots.

% x coordinate of CoM position. 
figure(nx+nu+1)
hold on
plot(Tcell{end},Z{end}(2,:),'b-',Tcell{end},Z{end}(2,:),'r-','LineWidth',2);
plot(Tcell{end},Z{end}(2,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_1^y}(t) \: \& \: \overline{r_1^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% y coordinate of CoM position. 
figure(nx+nu+2)
hold on
plot(Tcell{end},Z{end}(3,:),'b-',Tcell{end},Z{end}(3,:),'r-','LineWidth',2);
plot(Tcell{end},Z{end}(3,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_2^y}(t) \: \& \: \overline{r_2^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Position of CoM. 
figure(nx+nu+3)
hold on
plot(Z{end}(2,:),Z{end}(3,:),'b-',Z{end}(2,:),Z{end}(3,:),'r-','LineWidth',2);
plot(Z{end}(2,:),Z{end}(3,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_{1,2}^y}(t) \: \& \: \overline{r_{1,2}^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% x coordinate of CoM velocity. 
figure(nx+nu+4)
hold on
plot(Tcell{end},Z{end}(5,:),'b-',Tcell{end},Z{end}(5,:),'r-','LineWidth',2);
plot(Tcell{end},Z{end}(5,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_3^y}(t) \: \& \: \overline{r_3^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% y coordinate of CoM velocity. 
figure(nx+nu+5)
hold on
plot(Tcell{end},Z{end}(6,:),'b-',Tcell{end},Z{end}(6,:),'r-','LineWidth',2);
plot(Tcell{end},Z{end}(6,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_4^y}(t) \: \& \: \overline{r_4^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% CoM velocity.
figure(nx+nu+6)
hold on
plot(Z{end}(5,:),Z{end}(6,:),'b-',Z{end}(5,:),Z{end}(6,:),'r-','LineWidth',2);
plot(Z{end}(5,:),Z{end}(6,:),'color','[0,0.5,0]','LineWidth',2);
txt5 = '$\underline{r_{3,4}^y}(t) \: \& \: \overline{r_{3,4}^y}(t)\: \mathrm{for} \: t \in T_s$';
legend({txt1,txt2,txt5},'Interpreter','Latex','Location','Best')

% Plot CoM trajectory.
for i = 1:nx
    figure(nx+nu+i)
    for j = 1:nexp
        if i==1
            % xCoM(t)
            plot(Tcell{j},Z{j}(2,:),'b-','LineWidth',2);
        elseif i==2
            % yCoM(t)
            plot(Tcell{j},Z{j}(3,:),'b-','LineWidth',2);
        elseif i==3
            % xCoM(t) vs. yCoM(t)
            plot(Z{j}(2,:),Z{j}(3,:),'b-','LineWidth',2);
        elseif i==4
            % vxCoM(t)
            plot(Tcell{j},Z{j}(5,:),'b-','LineWidth',2);
        elseif i==5
            % vyCoM(t)
            plot(Tcell{j},Z{j}(6,:),'b-','LineWidth',2);
        else
            % vxCoM(t) vs. vyCoM(t)
            plot(Z{j}(5,:),Z{j}(6,:),'b-','LineWidth',2);
        end
    end
end

% Lower and upper bounds in the space of z. 
Zlbnd = Zbounds(:,:,1);
Zubnd = Zbounds(:,:,2);

% Plot reference CoM trajectory, and add labels.
for i = 1:nx
    figure(nx+nu+i)
    if i==1
        % x coordinate of CoM position. 
        plot(Tcell{end},Z{end}(2,:),'r-','LineWidth',2);
        plot(tspan,Zlbnd(:,2),tspan,Zubnd(:,2),'color','[0,0.5,0]','LineWidth',2);
        xlim([Tcell{end}(1),Tcell{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$x_{CoM}\left(t\right)\:\left[m\right]$','Interpreter','Latex')
    elseif i==2
        % y coordinate of CoM position. 
        plot(Tcell{end},Z{end}(3,:),'r-','LineWidth',2);
        plot(tspan,Zlbnd(:,3),tspan,Zubnd(:,3),'color','[0,0.5,0]','LineWidth',2);
        xlim([Tcell{end}(1),Tcell{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$y_{CoM}\left(t\right)\:\left[m\right]$','Interpreter','Latex')
    elseif i==3
        % CoM position. 
        plot(Z{end}(2,:),Z{end}(3,:),'r-','LineWidth',2);
        plot(Zlbnd(:,2),Zlbnd(:,3),Zubnd(:,2),Zubnd(:,3),[Zlbnd(1,2),Zubnd(1,2),Zubnd(1,2)],[Zlbnd(1,3),Zlbnd(1,3),Zubnd(1,3)],[Zlbnd(end,2),Zlbnd(end,2),Zubnd(end,2)],[Zlbnd(end,3),Zubnd(end,3),Zubnd(end,3)],'color','[0,0.5,0]','LineWidth',2);
        ylim([0.6,1.05]);
        xlabel('$x_{CoM}\:\left[m\right]$','Interpreter','Latex')
        ylabel('$y_{CoM}\:\left[m\right]$','Interpreter','Latex')
        axis equal
    elseif i==4
        % x coordinate of CoM velocity. 
        plot(Tcell{end},Z{end}(5,:),'r-','LineWidth',2);
        plot(tspan,Zlbnd(:,5),tspan,Zubnd(:,5),'color','[0,0.5,0]','LineWidth',2);
        xlim([Tcell{end}(1),Tcell{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$\dot{x}_{CoM}\left(t\right)\:\left[m/s\right]$','Interpreter','Latex')
    elseif i==5
        % y coordinate of CoM velocity. 
        plot(Tcell{end},Z{end}(6,:),'r-','LineWidth',2);
        plot(tspan,Zlbnd(:,6),tspan,Zubnd(:,6),'color','[0,0.5,0]','LineWidth',2);
        xlim([Tcell{end}(1),Tcell{end}(end)]);
        xlabel('$t\:\left[s\right]$','Interpreter','Latex')
        ylabel('$\dot{y}_{CoM}\left(t\right)\:\left[m/s\right]$','Interpreter','Latex')
    else
        % Velocity of CoM.
        [~,idx] = max(Zubnd(:,6));
        plot(Z{end}(5,:),Z{end}(6,:),'r-','LineWidth',2);
        plot(Zlbnd(:,5),Zlbnd(:,6),Zubnd(:,5),Zubnd(:,6),[Zlbnd(idx,5),Zlbnd(idx,5),Zubnd(idx,5)],[Zlbnd(idx,6),Zubnd(idx,6),Zubnd(idx,6)],[Zlbnd(end,5),Zubnd(end,5),Zubnd(end,5)],[Zlbnd(end,6),Zlbnd(end,6),Zubnd(end,6)],'color','[0,0.5,0]','LineWidth',2);
        ylim([-0.02,0.16]);
        xlabel('$\dot{x}_{CoM}\:\left[m/s\right]$','Interpreter','Latex')
        ylabel('$\dot{y}_{CoM}\:\left[m/s\right]$','Interpreter','Latex')
        axis equal
    end
    grid()
    % Export figure.
    if store
        if strcmp(plat,'PC')
            fname = [dirname,'\z', num2str(i), '.png'];
        elseif strcmp(plat,'MAC')
            fname = [dirname,'/z', num2str(i), '.png'];
        else
            error('No supported platform.');
        end
        hgexport(gcf,fname,s);
    end
end

%% Obtain successors at sample times from simulations.

% Arrays for storing succesors at sampled times. 
Xsuc = zeros(nexp,nx,nover);
Usuc = zeros(nexp,nu,nover);
Zsuc = zeros(nexp,nx,nover);

% Compute succesors of states at sampled times.
for k = 1:nover
    for i = 1:nexp
        for j = 1:nx
            Xsuc(i,j,k) = csapi(Tcell{i},Xcell{i}(:,j),tspan(k));
        end
    end
end

% Compute succesors of inputs at sampled times.
for k = 1:nover
    for i = 1:nexp
        for j = 1:nu
            Usuc(i,j,k) = csapi(Tcell{i},Ucell{i}(:,j),tspan(k));
        end
    end
end

% Compute succesors of CoM outputs at sampled times.
for k = 1:nover
    for i = 1:nexp
        Zs = xpar2CoMpv(Xsuc(i,:,k)',pexp(i,:));
        Zsuc(i,:,k) = Zs';
    end
end

%% Plot over-approximations for the position, and velocity of the CoM at sampled times.

% Reference trajectory in the space of z.
zbar = xpar2CoMpv(xbar',pnom);

% Array for vertices of over-approximation boxes for the position of the CoM.
PosVert = zeros(nover,5,2);

% Bounds for outputs from z space bounds.
Ybounds = Zbounds(:,[2,3,5,6],:);

% Compute vertices of over-approximation boxes for the position of the CoM.
for i=1:nover
   PosVert(i,:,1) = [Ybounds(i,1,2),Ybounds(i,1,2),Ybounds(i,1,1),Ybounds(i,1,1),Ybounds(i,1,2)];
   PosVert(i,:,2) = [Ybounds(i,2,1),Ybounds(i,2,2),Ybounds(i,2,2),Ybounds(i,2,1),Ybounds(i,2,1)];
end

% Array for vertices of over-approximation boxes for the velocity of the CoM.
VelVert = zeros(nover,5,2);

% Compute vertices of over-approximation boxes for the velocity of the CoM.
for i=1:nover
   VelVert(i,:,1) = [Ybounds(i,3,2),Ybounds(i,3,2),Ybounds(i,3,1),Ybounds(i,3,1),Ybounds(i,3,2)];
   VelVert(i,:,2) = [Ybounds(i,4,1),Ybounds(i,4,2),Ybounds(i,4,2),Ybounds(i,4,1),Ybounds(i,4,1)];
end

% Figure for position of the CoM. 
figure()

% Plot reference trajectory for the position of the CoM. 
plot(zbar(2,1),zbar(3,1),'b.',zbar(2,:),zbar(3,:),'r-','LineWidth',2);
hold all 

% Plot over-approximation boxes for sampled times. 
for i = 1:nover
   plot(PosVert(i,:,1),PosVert(i,:,2),'LineWidth',2)
end

% Add legends to plot.
CoMLegend = cell(1,nover+2);
CoMLegend{1} = 'Successors for $p \in \mathcal{P}_s$';
CoMLegend{2} = 'Reference Trajectory';
for i = 3:nover+2
    CoMLegend{i} = ['$\underline{r_{1,2}^y}(', num2str(tspan(i-2)), ') \: \& \: \overline{r_{1,2}^y}(', num2str(tspan(i-2)), ')$'];
end
legend(CoMLegend,'Interpreter','Latex','Location','Best');

% Plot position of CoM successors at sampled times.
for i = 1:nover
    plot(Zsuc(:,2,i),Zsuc(:,3,i),'b.','LineWidth',2)
end

% Axis labels. 
axis equal
ylim([0.6, 1.05])
grid()
xlabel('$x_{CoM}\:[m]$','Interpreter','Latex')
ylabel('$y_{CoM}\:[m]$','Interpreter','Latex')

% Export figure.
if store
    if strcmp(plat,'PC')
        fname = [dirname,'\CoMPosition.png'];
    elseif strcmp(plat,'MAC')
        fname = [dirname,'/CoMPosition.png'];
    else
        error('No supported platform.');
    end
    hgexport(gcf,fname,s);
end

% Figure for velocity of the CoM.
figure()

% Plot reference trajectory for the velocity of the CoM. 
plot(Zsuc(1,5,i),Zsuc(1,6,i),'b.',zbar(5,:),zbar(6,:),'r-','LineWidth',2);
hold all 

% Plot over-approximation boxes for sampled times.
for i = 1:nover
   plot(VelVert(i,:,1),VelVert(i,:,2),'LineWidth',2)
end

% Add legends to plot.
CoMVelLegend = cell(1,nover+2);
CoMVelLegend{1} = 'Successors for $p \in \mathcal{P}_s$';
CoMVelLegend{2} = 'Reference Trajectory';
for i = 3:nover+2
    CoMVelLegend{i} = ['$\underline{r_{3,4}^y}(', num2str(tspan(i-2)), ') \: \& \: \overline{r_{3,4}^y}(', num2str(tspan(i-2)), ')$'];
end
legend(CoMVelLegend,'Interpreter','Latex','Location','Best');

% Plot velocity of CoM successors for sampled times.  
for i = 1:nover
    plot(Zsuc(:,5,i),Zsuc(:,6,i),'b.','LineWidth',2)
end

% Add axis labels. 
axis equal
ylim([-0.01, 0.16])
grid()
xlabel('$\dot{x}_{CoM}\:[m/s]$','Interpreter','Latex')
ylabel('$\dot{y}_{CoM}\:[m/s]$','Interpreter','Latex')

% Export figure.
if store
    if strcmp(plat,'PC')
        fname = [dirname,'\CoMVelocity.png'];
    elseif strcmp(plat,'MAC')
        fname = [dirname,'/CoMVelocity.png'];
    else
        error('No supported platform.');
    end
    hgexport(gcf,fname,s);
end

%% Plot over-approximations for the states at each of the sampled times using phase portraits. 

% Cell array for vertices of over-approximation boxes.
XVert = cell(1,3);
for i=1:3
    XVert{i} = zeros(nover,5,2);
end

% Compute vertices of over-approximation boxes for phase portraits.
for j=1:3
    for i=1:nover
        XVert{j}(i,:,1) = [Xbounds(i,j,2),Xbounds(i,j,2),Xbounds(i,j,1),Xbounds(i,j,1),Xbounds(i,j,2)];
        XVert{j}(i,:,2) = [Xbounds(i,j+3,1),Xbounds(i,j+3,2),Xbounds(i,j+3,2),Xbounds(i,j+3,1),Xbounds(i,j+3,1)];
    end
end

% Figures for over-approximations of the state. 
for j=1:3
    figure()
    
    % Plot reference trajectory for the state.
    plot(xbar(1,j)*180/pi,xbar(1,j+3)*180/pi,'b.',xbar(:,j)*180/pi,xbar(:,j+3)*180/pi,'r-','LineWidth',2);
    hold all
    
    % Plot over-approximation boxes for sampled times.
    for i = 1:nover
        plot(XVert{j}(i,:,1)*180/pi,XVert{j}(i,:,2)*180/pi,'LineWidth',2)
    end
    
    % Add legends to plot.
    XLegend = cell(1,nover+2);
    XLegend{1} = 'Successors for $p \in \mathcal{P}_s$';
    XLegend{2} = 'Reference Trajectory';
    for i = 3:nover+2
        XLegend{i} = ['$\underline{r_{',num2str(j),',',num2str(j+3),'}^x}(', num2str(tspan(i-2)), ') \: \& \: \overline{r_{',num2str(j),',',num2str(j+3),'}^x}(', num2str(tspan(i-2)), ')$'];
    end
    legend(XLegend,'Interpreter','Latex','Location','Best');
    
    % Plot state successors under parameter uncertainty for sampled times.
    for i = 1:nover
        plot(Xsuc(:,j,i)*180/pi,Xsuc(:,j+3,i)*180/pi,'b.','LineWidth',2)
    end
    
    % Axis limits and labels.
    if j ==1
        ylim([-4, 6]);
    elseif j==2 
        xlim([-95, 0]);
    else 
        xlim([0, 110]);
    end
    axis equal
    grid()
    xlabel(['$\theta_{',num2str(j),'}\:[^\circ]$'],'Interpreter','Latex')
    ylabel(['$\dot{\theta}_{',num2str(j),'}\:[^\circ/s]$'],'Interpreter','Latex')
    
    % Export figure. 
    if store
        if strcmp(plat,'PC')
            fname = [dirname,'\PhasePortrait', num2str(j), '.png'];
        elseif strcmp(plat,'MAC')
            fname = [dirname,'/PhasePortrait', num2str(j), '.png'];
        else
            error('No supported platform.');
        end
        hgexport(gcf,fname,s);
    end
end

%% Plot over-approximations for the input at each of the sampled times.  

% Cell array for vertices of input over-approximation boxes.
UVert = cell(1,2);
for i=1:2
    UVert{i} = zeros(nover,5,2);
end

% Compute vertices of over-approximation boxes.
for i=1:nover
    % For torques.
    UVert{1}(i,:,1) = [Ubounds(i,1,2),Ubounds(i,1,2),Ubounds(i,1,1),Ubounds(i,1,1),Ubounds(i,1,2)];
    UVert{1}(i,:,2) = [Ubounds(i,2,1),Ubounds(i,2,2),Ubounds(i,2,2),Ubounds(i,2,1),Ubounds(i,2,1)];
    % For force.
    UVert{2}(i,:,1) = [Ubounds(i,3,2),Ubounds(i,3,2),Ubounds(i,3,1),Ubounds(i,3,1),Ubounds(i,3,2)];
    UVert{2}(i,:,2) = [Ubounds(i,4,1),Ubounds(i,4,2),Ubounds(i,4,2),Ubounds(i,4,1),Ubounds(i,4,1)];
end

% Figure for torques at the hips, and shoulders.
figure()

% Plot reference trajectory for torques at the hips, and shoulders.
plot(Usuc(1,1,1),Usuc(1,2,1),'b.',ubar(:,1),ubar(:,2),'r-','LineWidth',2);
hold all

% Plot over-approximation boxes for sampled times.
for i = 1:nover
    plot(UVert{1}(i,:,1),UVert{1}(i,:,2),'LineWidth',2)
end

% Add legends to plot.
ULegend = cell(1,nover+2);
ULegend{1} = 'Successors for $p \in \mathcal{P}_s$';
ULegend{2} = 'Reference Trajectory';
for i = 3:nover+2
    ULegend{i} = ['$\underline{r_{1,2}^u}(', num2str(tspan(i-2)), ') \: \& \: \overline{r_{1,2}^x}(', num2str(tspan(i-2)), ')$'];
end
legend(ULegend,'Interpreter','Latex','Location','Best');

% Plot torque successors for sampled times.
for i = 1:nover
    plot(Usuc(:,1,i),Usuc(:,2,i),'b.','LineWidth',2)
end

% Axis limits and labels.
ylim([-160, 30])
axis equal
grid()
xlabel('$\tau_{h}\:[N.m]$','Interpreter','Latex')
ylabel('$\tau_{s}\:[N.m]$','Interpreter','Latex')

% Export figure. 
if store
    if strcmp(plat,'PC')
        fname = [dirname,'\TorquePortrait.png'];
    elseif strcmp(plat,'MAC')
        fname = [dirname,'/TorquePortrait.png'];
    else
        error('No supported platform.');
    end
    hgexport(gcf,fname,s);
end

% Figure for force at the shoulders.
figure()

% Plot reference trajectory for the states.
plot(Usuc(1,3,1),Usuc(1,4,1),'b.',ubar(:,3),ubar(:,4),'r-','LineWidth',2);
hold all

% Plot over-approximation boxes for sampled times.
for i = 1:nover
    plot(UVert{2}(i,:,1),UVert{2}(i,:,2),'LineWidth',2)
end

% Add legends to plot.
ULegend = cell(1,nover+2);
ULegend{1} = 'Successors for $p \in \mathcal{P}_s$';
ULegend{2} = 'Reference Trajectory';
for i = 3:nover+2
    ULegend{i} = ['$\underline{r_{3,4}^u}(', num2str(tspan(i-2)), ') \: \& \: \overline{r_{3,4}^u}(', num2str(tspan(i-2)), ')$'];
end
legend(ULegend,'Interpreter','Latex','Location','Best');

% Plot force successors for sampled times.
for i = 1:nover
    plot(Usuc(:,3,i),Usuc(:,4,i),'b.','LineWidth',2)
end

% Axis labels.
grid()
xlabel('$F_{x}\:[N]$','Interpreter','Latex')
ylabel('$F_{y}\:[N]$','Interpreter','Latex')

% Export figure. 
if store
    if strcmp(plat,'PC')
        fname = [dirname,'\ForcePortrait.png'];
    elseif strcmp(plat,'MAC')
        fname = [dirname,'/ForcePortrait.png'];
    else
        error('No supported platform.');
    end
    hgexport(gcf,fname,s);
end