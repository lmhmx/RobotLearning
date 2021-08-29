clc; close all;clear;

%Initial conditions:
p.init = [pi/3    0.0    pi/3  0.0 pi/3 0.0]';

p.g = 9.81;
p.m1 = 1; %Mass of link 1.
p.m2 = 1; %Mass of link 2.
p.m3=1;
p.l1 = 1; %Total length of link 1.
p.l2 = 1; %Total length of link 2.
p.l3=1;
p.d1 = p.l1/2; %Center of mass distance along link 1 from the fixed joint.
p.d2 = p.l2/2; %Center of mass distance along link 2 from the fixed joint.
p.d3 = p.l3/2;
p.I1 = 1/12*p.m1*p.l1^2; %Moment of inertia of link 1 about COM
p.I2 = 1/12*p.m2*p.l2^2; %Moment of inertia of link 2 about COM
p.I3 = 1/12*p.m3*p.l3^2;

% endZ = ForwardKin(p.l1,p.l2,p.l3,p.init(1),p.init(3),p.init(5));
% x0 = endZ(1); %End effector initial position in world frame.
% y0 = endZ(2);
p.Fx = 0;
p.Fy = 0;

%%%%%%%% Control Parameters %%%%%%%%

%Controller Gains
p.Kp = 30;
p.Kd = 10;

%Single target:

p.h=0.001;
p.xt=@(t)(2*sin(t/8*2*pi+pi/3));
p.yt=@(t)(3*cos(t/8*2*pi+pi/3));

p.xtarget = p.xt(0); %What points are we shooting for in WORLD SPACE?
p.ytarget = p.yt(0);
%显示上面1000个点
p.totolNum=100;
p.trace = zeros(p.totolNum,2);
%当前点的个数，会取余，这样能够将原来的轨迹覆盖掉
p.Currentnum = 1;

Plotter(p) %Integration is done in realtime using symplectic euler like we did in the CS animation class.
function Plotter(p)
    close all

    %Playback speed:
    % playback = p.animationSpeed;

    %Name the whole window and define the mouse callback function
    f = figure;
    %set(f,'WindowButtonMotionFcn','','WindowButtonDownFcn',@ClickDown,'WindowButtonUpFcn',@ClickUp,'KeyPressFc',@KeyPress);

    figData.xtarget = [];
    figData.ytarget = [];
    figData.Fx = [];
    figData.Fy = [];
    figData.xend = [];
    figData.yend = [];
    figData.fig = f;
    figData.tarControl = true;

    %%%%%%%% 1st Subplot -- the pendulum animation %%%%%%%
    figData.simArea = subplot(1,1,1); %Eliminated other subplots, but left this for syntax consistency.
    axis equal
    hold on

    %Create pendulum link1 object:
    width1 = p.l1*0.05;
    xdat1 = 0.5*width1*[-1 1 1 -1];
    ydat1 = p.l1*[0 0 1 1];
    link1 = patch(xdat1,ydat1, [0 0 0 0],'r');

    %Create pendulum link2 object:
    width2 = p.l2*0.05;
    xdat2 = 0.5*width2*[-1 1 1 -1];
    ydat2 = p.l2*[0 0 1 1];
    link2 = patch(xdat2,ydat2, [0 0 0 0],'b');
    
    %Create pendulum link2 object:
    width3 = p.l3*0.05;
    xdat3 = 0.5*width3*[-1 1 1 -1];
    ydat3 = p.l3*[0 0 1 1];
    link3 = patch(xdat3,ydat3, [0 0 0 0],'g');
    axis([-3.5 3.5 -3.6 3.6]);

    
    %Dots for the hinges:
    h1 = plot(0,0,'.k','MarkerSize',40); %First link anchor
    h2 = plot(0,0,'.k','MarkerSize',40); %link1 -> link2 hinge
    h3 = plot(0,0,'.k','MarkerSize',40);
    
    %Timer label:
    timer = text(-3.2,-3.2,'0.00','FontSize',28);

    %Torque meters on screen
    tmeter1 = text(0.6,-3.2,'0.00','FontSize',22,'Color', 'r');
    tmeter2 = text(2.2,-3.2,'0.00','FontSize',22,'Color', 'b');
    tmeter3 = text(3.2,-3,2,'0.00','Fontsize',22,'Color', 'g');
    %Target Pt.
    targetPt = plot(p.xtarget,p.ytarget,'xr','MarkerSize',30);
    tracePt = plot(p.trace(:,1),p.trace(:,2),'.');
    hold off

    %Make the whole window big for handy viewing:
    set(f, 'units', 'inches', 'position', [3 3 6 5])
    set(f,'Color',[1,1,1]);

    % Turn the axis off
    ax = get(f,'Children');
    set(ax,'Visible','off');

    %Animation plot loop -- Includes symplectic integration now.
    z1 = p.init;
    told = 0;

    set(f,'UserData',figData);
    
    tic %Start the clock
    while (ishandle(f))
        figData = get(f,'UserData');
        %%%% INTEGRATION %%%%
        tnew = toc;
        dt = tnew - told;
        
        %Old velocity and position
        xold = [z1(1),z1(3),z1(5)];
        vold = [z1(2),z1(4),z1(6)];

        %Call RHS given old state
        [zdot1, T1, T2, T3] = FullDyn(tnew,z1,p);
        
        vinter1 = [zdot1(1),zdot1(3),zdot1(5)];
        ainter = [zdot1(2),zdot1(4),zdot1(6)];

        vinter2 = vold + ainter*dt; %Update velocity based on old RHS call

        %Update position.
        xnew = xold + vinter2*dt;
        vnew = vinter2;

        z2 = [xnew(1) vnew(1) xnew(2) vnew(2) xnew(3) vnew(3)];

        z1 = z2;
        told = tnew;
        %%%%%%%%%%%%%%%%%%%%

        %If there are new mouse click locations, then set those as the new
        %target.
%         if ~isempty(figData.xtarget)
%             %p.xtarget = figData.xtarget;
%             p.xtarget = p.xt(tnew);
%         end
% 
%         if ~isempty(figData.ytarget)
%             %p.ytarget = figData.ytarget;
%             p.ytarget = p.yt(tnew);
%         end

        p.xtarget = p.xt(tnew);
        p.ytarget = p.yt(tnew);
        p.trace(p.Currentnum+1,:)=[p.xtarget,p.ytarget];
        p.Currentnum=mod(p.Currentnum+1,p.totolNum);
        set(tracePt,'xData',p.trace(:,1));
        set(tracePt,'yData',p.trace(:,2));
        set(targetPt,'xData',p.xtarget); %Change the target point graphically.
        set(targetPt,'yData',p.ytarget);

        %When you hit a key, it changes to force mode, where the mouse will
        %pull things.
        ra_e = ForwardKin(p.l1,p.l2,p.l3,z1(1),z1(3),z1(5));
        figData.xend = ra_e(1);
        figData.yend = ra_e(2);
        set(f,'UserData',figData);

        if ~isempty(figData.Fx)
            p.Fx = figData.Fx;
        end
        if ~isempty(figData.Fy)
            p.Fy = figData.Fy;
        end

        tstar = told; %Get the time (used during this entire iteration)

        %On screen timer.
        set(timer,'string',strcat(num2str(tstar,3),'s'))
        zstar = z1;%interp1(time,zarray,tstar); %Interpolate data at this instant in time.

        %Rotation matrices to manipulate the vertices of the patch objects
        %using theta1 and theta2 from the output state vector.
        rot1 = [cos(zstar(1)-pi/2), -sin(zstar(1)-pi/2);
            sin(zstar(1)-pi/2),cos(zstar(1)-pi/2)]*[xdat1;ydat1];
        set(link1,'xData',rot1(1,:))
        set(link1,'yData',rot1(2,:))
        set(h2,'xData',p.l1*cos(zstar(1)));
        set(h2,'yData',p.l1*sin(zstar(1)));
        
        rot2 = [cos(zstar(3)+zstar(1)-pi/2), -sin(zstar(3)+zstar(1)-pi/2); 
            sin(zstar(3)+zstar(1)-pi/2),cos(zstar(3)+zstar(1)-pi/2)]*[xdat2;ydat2];
        set(link2,'xData',rot2(1,:)+(rot1(1,3)+rot1(1,4))/2) %We want to add the midpoint of the far edge of the first link to all points in link 2.
        set(link2,'yData',rot2(2,:)+(rot1(2,3)+rot1(2,4))/2)
        set(h3,'xData',p.l1*cos(zstar(1))+p.l2*cos(zstar(1)+zstar(3)));
        set(h3,'yData',p.l1*sin(zstar(1))+p.l2*sin(zstar(1)+zstar(3)));
        rot3 = [cos(zstar(5)+zstar(3)+zstar(1)-pi/2), -sin(zstar(5)+zstar(3)+zstar(1)-pi/2); 
            sin(zstar(5)+zstar(3)+zstar(1)-pi/2),cos(zstar(5)+zstar(3)+zstar(1)-pi/2)]*[xdat3;ydat3];
        
        set(link3,'xData',rot3(1,:)+p.l1*cos(zstar(1))+p.l2*cos(zstar(1)+zstar(3))) %We want to add the midpoint of the far edge of the first link to all points in link 2.
        set(link3,'yData',rot3(2,:)+p.l1*sin(zstar(1))+p.l2*sin(zstar(1)+zstar(3)))

        %Change the hinge dot location
        
        
        
        
        %Show torques on screen (text only atm) update for time series later.
        set(tmeter1,'string',strcat(num2str(T1,2),' Nm'));
        set(tmeter2,'string',strcat(num2str(T2,2),' Nm'));
        set(tmeter3,'string',strcat(num2str(T3,2),' Nm'));
        drawnow;
    end
    disp("close")
end



