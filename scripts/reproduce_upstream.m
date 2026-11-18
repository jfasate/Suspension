% Run from project root using MATLAB R2025b or newer for full Simulink replay.
% Copies into a temporary folder because the original script clears workspace.
project_root = pwd;
reference_dir = tempname;
mkdir(reference_dir);
copyfile(fullfile(project_root,'third_party','upstream','*'),reference_dir);
cd(reference_dir);
FULL_CAR_MODEL;
out = sim('FULL_CAR_SIM','StopTime','1','ReturnWorkspaceOutputs','on');
disp('UPSTREAM_SIMULATION_COMPLETED');
