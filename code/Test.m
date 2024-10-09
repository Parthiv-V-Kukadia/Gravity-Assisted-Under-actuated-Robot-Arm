% Write all necessary commands to verify your requirement in this script.
%
% In particular, one should be able to run this script in a folder 
% that contains "Controller.m" and "DesignProblem02.m" and should see
% evidence that would allow him or her to conclude (as described in your report)
% that your requirement has been satisfied.
%
% At minimum, this script should run your simulation at least once with no
% error (i.e., the simulation figure always says CONTROLLER: ON).
%
% Here is a simple template. Please modify as you wish.

DesignProblem02('Controller','datafile','data.mat','tstop',30,'diagnostics',true,'initial',[.8;.8;.1;.1])

% Please be considerate of your reviewers' time and memory when making
% choices about what to include here. Please do not write scripts that
% create massive matrices or take 10+ minutes to run once.
