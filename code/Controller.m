function func = Controller
% INTERFACE
%
%   sensors
%       .t          (time)
%       .q1         (first joint angle)
%       .q2         (second joint angle, relative to the first)
%       .v1         (first joint velocity)
%       .v2         (second joint velocity)
%
%   references
%       .q2         (desired angle of second joint)
%
%   parameters
%       .tStep      (time step)
%       .tauMax     (maximum torque that can be applied)
%       .symEOM     (symbolic description of equations of motion)
%       .numEOM     (numerc description of equations of motion)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .tau1       (torque applied about first joint)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [data] = initControlSystem(parameters,data)

%
% Here is a good place to initialize things...
%Matrices solved for in EOMs file
%at q1_e = 0.5

data.K= [12.2785  -20.0592    3.5406   -5.5970];
data.Kref=-3.0132;


end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors, references, parameters, data)
%if sensors.t <= 12
%    data.r=0.4;
%else
%    data.r = 0;
%end

data.r = 0.03; %sensors.q2;
data.Kint = 4;
data.X=[sensors.q1; sensors.q2; sensors.v1; sensors.v2];
data.V=(sensors.q2 - data.r);

%u= -K*x + kRef*r + kInt*v(t)
actuators.tau1 = -data.K*data.X +data.Kref*data.r + data.Kint*data.V;
end
