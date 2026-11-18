function summary = run_upstream_reference(output_dir, duration)
% Run the preserved upstream model in an isolated copy (MATLAB R2025b+).
% The original initialization uses CLEAR; execute it in base workspace so the
% runner's parameters, cleanup handler and output directory remain intact.
project_root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(output_dir)
    output_dir = fullfile(project_root, 'results', 'upstream_simulink');
end
if nargin < 2
    duration = 10;
end
validateattributes(duration, {'numeric'}, {'scalar','positive','finite'});
if ~isfolder(output_dir)
    mkdir(output_dir);
end
% Resolve relative paths before changing into the isolated reference folder.
[ok, attr] = fileattrib(output_dir);
assert(ok, 'Unable to resolve output directory');
output_dir = attr.Name;
reference_dir = tempname;
mkdir(reference_dir);
copyfile(fullfile(project_root,'third_party','upstream','*'), reference_dir);
previous_dir = pwd;
cleanup = onCleanup(@() restore_directory(previous_dir)); %#ok<NASGU>
cd(reference_dir);
diary(fullfile(output_dir, 'execution.log'));
fprintf('UPSTREAM_REFERENCE_DIRECTORY: %s\n', reference_dir);
fprintf('MATLAB_RELEASE: %s\n', version('-release'));
evalin('base', 'FULL_CAR_MODEL;');
simulation_output = sim('FULL_CAR_SIM', 'StopTime', num2str(duration), ...
    'ReturnWorkspaceOutputs', 'on');
save(fullfile(output_dir,'simulation_output.mat'), 'simulation_output', '-v7.3');
names = who(simulation_output);
summary = struct('release', version('-release'), 'matlab_version', version, ...
    'upstream_revision', '098f9fdbc5c311d64110f1da30d791644c9f4a77', ...
    'requested_duration_s', duration, 'signals', struct, 'completed', true);
for index = 1:numel(names)
    name = names{index};
    value = simulation_output.get(name);
    if isa(value, 'timeseries')
        data = value.Data;
    elseif isnumeric(value)
        data = value;
    elseif isstruct(value) && isfield(value,'signals') && isfield(value.signals,'values')
        data = value.signals.values;
    else
        continue
    end
    assert(all(isfinite(data(:))), 'Nonfinite output in %s', name);
    summary.signals.(name) = struct('class',class(value), 'samples',size(data,1), ...
        'peak_absolute_value',max(abs(data(:))));
end
file_id = fopen(fullfile(output_dir,'summary.json'), 'w');
assert(file_id >= 0, 'Unable to write summary.json');
file_cleanup = onCleanup(@() fclose(file_id)); %#ok<NASGU>
fprintf(file_id, '%s\n', jsonencode(summary, PrettyPrint=true));
disp('UPSTREAM_SIMULATION_COMPLETED');
disp(summary);
diary off;
end

function restore_directory(previous_dir)
diary off;
if bdIsLoaded('FULL_CAR_SIM')
    bdclose('FULL_CAR_SIM');
end
cd(previous_dir);
end
