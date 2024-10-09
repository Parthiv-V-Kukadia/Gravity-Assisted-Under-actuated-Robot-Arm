function DesignProblem02(controller,varargin)
% DesignProblem02   run simulation of robot arm
%
%   DesignProblem02('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%   DesignProblem02('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'team' : a name (e.g., 'FirstName LastName') that, if defined,
%                    will appear on the figure window
%
%           'datafile' : a filename (e.g., 'data.mat') where, if defined,
%                        data will be logged and saved
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'controllerdatatolog' : a cell array (e.g., {'y','xhat'}) with
%                                   the names of fields in controller.data -
%                                   if 'datafile' is defined (so data is
%                                   logged), then values in these fields will
%                                   also be logged and saved
%
%           'diagnostics' : a flag - true or false (default) that, if true,
%                           will show plots of state and actuator values
%
%           'tStop' : the time at which the simulation will stop (a
%                     positive number) - defaults value is 30
%
%           'disturbance' : a flag - true or false (default) that, if true,
%                           will add an unknown disturbance to each torque
%
%           'reference' : a function of time that specifies a reference
%                         value of q2 - for example, the following choice
%                         would specify a reference value q2(t) = sin(t):
%                           @(t) sin(t)
%
%           'initial' : a 4x1 matrix [q1 q2 v1 v2] that specifies the
%                       initial joint angles and joint velocities - by
%                       default, these values are:
%                       	[0.1; 0.01; 0.01; 0.01].*randn(4,1)
%
%           'display' : a flag...
%
%                       - If true, it will clear the current figure and
%                         will show the simulation. To quite, type 'q' when
%                         this figure is in the foreground.
%
%                       - If false, it will not show any graphics and will
%                         run the simulation as fast as possible (not in
%                         real-time).

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'controller',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'team',[],@ischar);
addParameter(p,'datafile',[],@ischar);
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'controllerdatatolog',[],@iscell);
addParameter(p,'diagnostics',false,@islogical);
addParameter(p,'tStop',30,@(x) isscalar(x) && isnumeric(x) && (x>0));
addParameter(p,'disturbance',false,@islogical);
addParameter(p,'reference',@(x)0,@(x) isa(x,'function_handle'));
addParameter(p,'initial',[0.1; 0.01; 0.01; 0.01].*randn(4,1),...
                         @(x) validateattributes(x,{'numeric'},{'size',[4 1]}));
addParameter(p,'display',true,@islogical);
% - Apply input parser
parse(p,controller,varargin{:});
% - Extract parameters
process = p.Results;
% - Check that the 'controller' function exists
if (exist(process.controller,'file')~=2)
    error('Controller ''%s'' does not exist.',process.controller);
end
% - Check display
if ~process.display && ~isempty(process.moviefile)
    error('You cannot ask to save a movie with the display turned off.');
end
if ~process.display && ~isempty(process.snapshotfile)
    error('You cannot ask to save a snapshot with the display turned off.');
end
% - Enable all warnings
warning('on');

% Setup the simulation
[process,controller] = SetupSimulation(process);

% Run the simulation
RunSimulation(process,controller);


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)

% DEFINE CONSTANTS

% Constants related to simulation.
% - State time.
process.tStart = 0;
% - Time step.
process.tStep = 1/50;
% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','q1','q2','v1','v2'};

% Constants related to physical properties.
% - Dimensions of each link [x y z]
process.d0 = [0.4 0.6 0.6];
process.d1 = [1 1 1];
process.d2 = [5.5 1 0.2];
% - Relative position of each frame
sep = 0;
l1 = (process.d0(1)/2)+(process.d1(1)/2)+(sep);
l2 = (process.d1(3)/2)+(process.d2(3)/2)+(sep);
l3 = (process.d2(1)/2)-(process.d1(1)/2);
process.p1 = [l1; 0; 0];
process.p2 = [l3; 0; l2];
% - Mass
rho = 0.8;
[process.m1,process.J1in1] = getMassAndMomentOfInertia(rho,process.d1);
[process.m2,process.J2in2] = getMassAndMomentOfInertia(rho,process.d2);
% - Viscous friction
process.b1 = 0.5;
process.b2 = 1.5;
% - Gravity
process.g = 9.81;
% - EOM
filename = 'DesignProblem02_EOMs.mat';
if (exist(filename,'file')==2)
    fprintf(1,'Loading EOMs from file (delete %s to start fresh).\n',filename);
    load(filename);
else
    [symEOM,numEOM] = GetEOM(process.p1,process.p2,...
                             process.m1,process.m2,...
                             process.J1in1,process.J2in2,...
                             process.b1,process.b2,...
                             process.g);
	fprintf(1,'Saving EOMs to file (load %s to work with them).\n',filename);
	save('DesignProblem02_EOMs.mat','symEOM','numEOM');
end
process.symEOM = symEOM;
process.numEOM = numEOM;
% - Maximum torque
process.tauMax = 20;
% - Disturbance torque
if process.disturbance
    tau = (-1+2*rand);
    tau = tau + 1*sign(tau);
    process.disturbanceTorque = tau;
end

% DEFINE VARIABLES

% Time
process.t = 0;
% Joint angles
process.q1 = process.initial(1,1);
process.q2 = process.initial(2,1);
% Joint velocities
process.v1 = process.initial(3,1);
process.v2 = process.initial(4,1);

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
controller = eval(process.controller);
controller.name = process.controller;
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','tauMax','symEOM','numEOM'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% Status
controller.running = true;
% Init
tic
try
    [controller.data] = ...
        controller.init(controller.parameters, ...
                        controller.data);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tInit = toc;
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
end

function controller = RunController(controller)
if (controller.running)
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.run(controller.sensors, ...
                              controller.references, ...
                              controller.parameters, ...
                              controller.data);
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tRun = toc;
else
    controller.tRun = 0;
end
end

function references = GetReferences(process)
try
    references = struct('q2',process.reference(process.t));
catch exception
    warning(['The ''references'' function that was passed to\n' ...
             'DesignProblem02 threw the following error:\n\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Leaving references unchanged.\n'],getReport(exception));
    references = struct('q2',0);
end
end

function [m,J] = getMassAndMomentOfInertia(rho,d)
m = rho*d(1)*d(2)*d(3);
J = (m/12)*diag([d(2)^2+d(3)^2,d(1)^2+d(3)^2,d(1)^2+d(2)^2]);
end

function [symEOM,numEOM] = GetEOM(p1,p2,m1,m2,J1in1,J2in2,b1,b2,g)

syms q1 q2 real
q = [q1; q2];
syms v1 v2 real
v = [v1; v2];
syms tau1 real
tau = [tau1; 0];

R1in0 = [1 0 0; 0 cos(q1) -sin(q1); 0 sin(q1) cos(q1)];
R2in1 = [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1];
R2in0 = R1in0*R2in1;

p2hat = [0 -p2(3) p2(2); p2(3) 0 -p2(1); -p2(2) p2(1) 0];

Jv1 = [0 0; 0 0; 0 0];
Jw1 = [1 0; 0 0; 0 0];

x1in2 = R2in1'*[1;0;0];
z2in2 = [0;0;1];

Jv2 = simplify(-R2in0*p2hat*[x1in2 z2in2]);
Jw2 = [x1in2 z2in2];

M = simplify((m1*Jv1'*Jv1+Jw1'*J1in1*Jw1)+(m2*Jv2'*Jv2+Jw2'*J2in2*Jw2));

C = sym(zeros(2));
for i=1:2
    for j=1:2
        for k=1:2
            gamma = diff(M(i,j),q(k))+diff(M(i,k),q(j))-diff(M(k,j),q(i));
            C(i,j) = C(i,j) + gamma*v(k);
        end
    end
end
C = simplify(C/2);

z0in0 = [0;0;1];
o2in0 = R1in0*p1+R2in0*p2;
V = m2*g*o2in0'*z0in0;
N = simplify([diff(V,q(1)); diff(V,q(2))]+diag([b1,b2])*v);

symEOM.M = M;
symEOM.C = C;
symEOM.N = N;
symEOM.tau = tau;

o1in0 = R1in0*p1;

numEOM.M = matlabFunction(symEOM.M,'Vars',[q1 q2]);
numEOM.C = matlabFunction(symEOM.C,'Vars',[q1 q2 v1 v2]);
numEOM.N = matlabFunction(symEOM.N,'Vars',[q1 q2 v1 v2]);
numEOM.tau = matlabFunction(symEOM.tau,'Vars',tau1);
numEOM.o1in0 = matlabFunction(o1in0,'Vars',[q1,q2]);
numEOM.R1in0 = matlabFunction(R1in0,'Vars',[q1,q2]);
numEOM.o2in0 = matlabFunction(o2in0,'Vars',[q1,q2]);
numEOM.R2in0 = matlabFunction(R2in0,'Vars',[q1,q2]);

end

function sensors = GetSensors(process)
sensors.t = process.t;
sensors.q1 = process.q1;
sensors.q2 = process.q2;
sensors.v1 = process.v1;
sensors.v2 = process.v2;
% Add noise
%   (nothing)
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.q1; process.q2; process.v1; process.v2];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.tau1];

% Bound input
for i=1:length(u)
    if (u(i) < -process.tauMax)
        u(i) = -process.tauMax;
    elseif (u(i) > process.tauMax)
        u(i) = process.tauMax;
    end
end

% Add disturbance
u = [u; 0];
if process.disturbance
    u(2) = u(2) + process.disturbanceTorque;
end
end

function process = Get_Process_From_TandX(t,x,process)
process.t = t;
process.q1 = x(1,1);
process.q2 = x(2,1);
process.v1 = x(3,1);
process.v2 = x(4,1);
end

function xdot = GetXDot(t,x,u,process)
% unpack x and u
q1 = x(1,1);
q2 = x(2,1);
v1 = x(3,1);
v2 = x(4,1);
tau1 = u(1,1);
tau2 = u(2,1);
% compute xdot
M = process.numEOM.M(q1,q2);
tau = process.numEOM.tau(tau1)+[0;tau2];
C = process.numEOM.C(q1,q2,v1,v2);
N = process.numEOM.N(q1,q2,v1,v2);
xdot = [v1;
        v2;
        inv(M)*(tau-C*[v1;v2]-N)];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'tau1'}))&&(length(fieldnames(actuators))==1)
    if isnumeric(actuators.tau1)
        if isscalar(actuators.tau1)
            iscorrect = true;
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('tau1',0);
end

function fig = UpdateFigure(process,controller,fig)
if (isempty(fig))
    % CREATE FIGURE

    % Clear the current figure.
    clf;

    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    if (controller.running)
        status = 'ON';
        color = 'g';
    else
        status = 'OFF';
        color = 'r';
    end
    fig.text.status=text(0.05,0.975,...
        sprintf('CONTROLLER: %s',status),...
        'fontweight','bold','fontsize',fs,...
        'color',color,'verticalalignment','top');
    fig.text.time=text(0.05,0.12,...
        sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop),...
        'fontsize',fs,'verticalalignment','top','fontname','monaco');
    fig.text.teamname=text(0.05,0.06,...
        sprintf('%s',process.team),...
        'fontsize',fs,'verticalalignment','top','fontweight','bold');

    % Create axes for diagnostic plots
    if process.diagnostics
        fig.x.axis = axes('position',[0.55,0.6,0.4,0.35],'fontsize',fs);
        axis([0,process.tStop,-pi/2,pi/2]);
        hold on;
        fig.x.q1 = plot(nan,nan,'linewidth',2);
        fig.x.q2 = plot(nan,nan,'linewidth',2);
        fig.x.v1 = plot(nan,nan,'linewidth',2);
        fig.x.v2 = plot(nan,nan,'linewidth',2);
        fig.x.r = plot(nan,nan,':','linewidth',3);
        fig.x.legend = legend({'q1','q2','v1','v2','q2 (desired)'});
        xlabel('time');

        fig.u.axis = axes('position',[0.55,0.1,0.4,0.35],'fontsize',fs);
        delta = 0.1*process.tauMax;
        axis([0,process.tStop,-process.tauMax-delta,process.tauMax+delta]);
        hold on;
        fig.u.tau1 = plot(nan,nan,'linewidth',3);
        fig.u.umin = plot([0 process.tStop],-process.tauMax*[1 1],...
                          'linewidth',1,'linestyle','--','color','k');
        fig.u.umax = plot([0 process.tStop],process.tauMax*[1 1],...
                          'linewidth',1,'linestyle','--','color','k');
        fig.u.legend = legend({'tau1'});
        xlabel('time');
    end

    % Create an axis for the view from frame 0.
    if process.diagnostics
        fig.view0.axis = axes('position',[0 0.02 0.5 1]);
        axis equal;
        axis([-6 6 -6 6 -6 6]);
    else
        fig.view0.axis = axes('position',[0 0 1 1]);
        axis equal;
        axis([-8 8 -8 8 -6 6]);
    end
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    axis manual;
    hold on;
    axis off;
    view([37.5,20]);
    box on;
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[0;0;2],'style','local');

    pFrame = [0 1 0 0;
              0 0 1 0;
              0 0 0 1];
    fig.geom.pFrame0_in0 = pFrame;
    fig.geom.pFrame1_in1 = pFrame;
    fig.geom.pFrame2_in2 = pFrame;
    [fig.geom.pLink0_in0,fig.geom.fLink0] = getBox(process.d0);
    [fig.geom.pLink1_in1,fig.geom.fLink1] = getBox(process.d1);
    [fig.geom.pLink2_in2,fig.geom.fLink2] = getBox(process.d2);
    
    o1in0 = process.numEOM.o1in0(process.q1,process.q2);
    R1in0 = process.numEOM.R1in0(process.q1,process.q2);
    o2in0 = process.numEOM.o2in0(process.q1,process.q2);
    R2in0 = process.numEOM.R2in0(process.q1,process.q2);
    
    fig.geom.pFrame1_in0 = Transform(o1in0,R1in0,fig.geom.pFrame1_in1);
    fig.geom.pFrame2_in0 = Transform(o2in0,R2in0,fig.geom.pFrame2_in2);
    fig.geom.pLink1_in0 = Transform(o1in0,R1in0,fig.geom.pLink1_in1);
    fig.geom.pLink2_in0 = Transform(o2in0,R2in0,fig.geom.pLink2_in2);
    
    fig.view0.frame0 = DrawFrame([],fig.geom.pFrame0_in0);
    fig.view0.frame1 = DrawFrame([],fig.geom.pFrame1_in0);
    fig.view0.frame2 = DrawFrame([],fig.geom.pFrame2_in0);
    fig.view0.link0 = DrawMesh([],fig.geom.pLink0_in0,fig.geom.fLink0,'y',0.9);
    fig.view0.link1 = DrawMesh([],fig.geom.pLink1_in0,fig.geom.fLink1,'y',0.9);
    fig.view0.link2 = DrawMesh([],fig.geom.pLink2_in0,fig.geom.fLink2,'y',0.9);

    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
end

% UPDATE FIGURE

set(fig.text.time,'string',sprintf('t = %6.2f / %6.2f\n',process.t,process.tStop));
if (controller.running)
    status = 'ON';
    color = 'g';
else
    status = 'OFF';
    color = 'r';
end
set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);

if process.diagnostics
    t = [get(fig.x.q1,'xdata') process.t];
    q1 = [get(fig.x.q1,'ydata') process.q1];
    set(fig.x.q1,'xdata',t,'ydata',q1);
    q2 = [get(fig.x.q2,'ydata') process.q2];
    set(fig.x.q2,'xdata',t,'ydata',q2);
    v1 = [get(fig.x.v1,'ydata') process.v1];
    set(fig.x.v1,'xdata',t,'ydata',v1);
    v2 = [get(fig.x.v2,'ydata') process.v2];
    set(fig.x.v2,'xdata',t,'ydata',v2);
    r = [get(fig.x.r,'ydata') controller.references.q2];
    set(fig.x.r,'xdata',t,'ydata',r);
    tau1 = [get(fig.u.tau1,'ydata') controller.actuators.tau1];
    set(fig.u.tau1,'xdata',t,'ydata',tau1);
end


o1in0 = process.numEOM.o1in0(process.q1,process.q2);
R1in0 = process.numEOM.R1in0(process.q1,process.q2);
o2in0 = process.numEOM.o2in0(process.q1,process.q2);
R2in0 = process.numEOM.R2in0(process.q1,process.q2);

fig.geom.pFrame1_in0 = Transform(o1in0,R1in0,fig.geom.pFrame1_in1);
fig.geom.pFrame2_in0 = Transform(o2in0,R2in0,fig.geom.pFrame2_in2);
fig.geom.pLink1_in0 = Transform(o1in0,R1in0,fig.geom.pLink1_in1);
fig.geom.pLink2_in0 = Transform(o2in0,R2in0,fig.geom.pLink2_in2);

fig.view0.frame1 = DrawFrame(fig.view0.frame1,fig.geom.pFrame1_in0);
fig.view0.frame2 = DrawFrame(fig.view0.frame2,fig.geom.pFrame2_in0);
fig.view0.link0 = DrawMesh(fig.view0.link0,fig.geom.pLink0_in0);
fig.view0.link1 = DrawMesh(fig.view0.link1,fig.geom.pLink1_in0);
fig.view0.link2 = DrawMesh(fig.view0.link2,fig.geom.pLink2_in0);

drawnow;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(process.moviefile))
    myV = VideoWriter(process.moviefile,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/process.tStep;
    open(myV);
end

% LOOP

% Loop until break.
tStart = tic;
while (1)

    % Update figure (create one if fig is empty).
    if (process.display)
        fig = UpdateFigure(process,controller,fig);
    end

    % Update data.
    if (~isempty(process.datafile))
        [process,controller] = UpdateDatalog(process,controller);
    end

    % If making a movie, store the current figure as a frame.
    if (~isempty(process.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end

    % Stop if time has reached its maximum.
    if ((process.t + eps >= process.tStop)||done)
        break;
    end

    % Update process (integrate equations of motion).
    [process,controller] = UpdateProcess(process,controller);

    % Wait if necessary, to stay real-time.
    if (process.display)
        while (toc(tStart)<process.t-process.tStart)
            % Do nothing
        end
    end

end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(process.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
if (~isempty(process.datafile))
    processdata = process.log.process; %#ok<NASGU>
    controllerdata = process.log.controller; %#ok<NASGU>
    save(process.datafile,'processdata','controllerdata');
end

% Save the snapshot, if necessary.
if (~isempty(process.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',process.snapshotfile);
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tInit',[],...
                                             'tRun',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write process data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
% Write controller data to log, if controller is running.
if controller.running
    process.log.controller.tInit = controller.tInit;
    process.log.controller.tRun(:,process.log.count) = ...
        controller.tRun;
    names = fieldnames(controller.sensors);
    for i=1:length(names)
        name = names{i};
        process.log.controller.sensors.(name)(:,process.log.count) = ...
            controller.sensors.(name);
    end
    names = fieldnames(controller.actuators);
    for i=1:length(names)
        name = names{i};
        process.log.controller.actuators.(name)(:,process.log.count) = ...
            controller.actuators.(name);
    end
    for i=1:length(process.controllerdatatolog)
        name = process.controllerdatatolog{i};
        try
            process.log.controller.data.(name)(:,process.log.count) = ...
                controller.data.(name);
        catch exception
            warning(['Saving element ''%s'' of data for controller\n',...
                     '     ''%s''',...
                     'threw the following error:\n\n' ...
                     '==========================\n' ...
                     '%s\n', ...
                     '==========================\n\n' ...
                     'Turning off controller and setting all\n' ...
                     'actuator values to zero.\n'],...
                     name,controller.name,getReport(exception));
            controller.actuators = ZeroActuators();
            controller.running = false;
            return
        end
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
controller = RunController(controller);
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end

function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end

function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end

function R = R_ZYX(theta)
R = RZ(theta(1))*RY(theta(2))*RX(theta(3));
end

function thetadot = GetAngularRates_ZYX(theta,w)
c2 = cos(theta(2));
s2 = sin(theta(2));
c3 = cos(theta(3));
s3 = sin(theta(3));
A = [   -s2     0       1;
        c2*s3   c3      0;
        c2*c3   -s3     0];
thetadot = A\w;
end

function wHat = wedge(w)
wHat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end

function p_inj = Transform(o_kinj,R_kinj,p_ink)
p_inj = zeros(size(p_ink));
for i=1:size(p_ink,2)
    p_inj(:,i) = o_kinj + R_kinj*p_ink(:,i);
end
end

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function mesh = DrawMesh(mesh,p,f,color,alpha)
if isempty(mesh)
    mesh = patch('Vertices',p','Faces',f,...
                 'FaceColor',color,'FaceAlpha',alpha,'EdgeAlpha',alpha);
else
    set(mesh,'vertices',p');
end
end

function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end
end

function [p,f] = getBox(d)
p = [[-1 1 1 -1 -1 1 1 -1];[-1 -1 1 1 -1 -1 1 1];[-1 -1 -1 -1 1 1 1 1]];
p = 0.5*repmat(reshape(d,3,1),1,8).*p;
f = [1 2 3 4;
     5 6 7 8;
     1 2 6 5;
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
