tool_path = fileparts(mfilename('fullpath'));
addpath(tool_path);
subfolders = {'curves', 'jacobians', 'kinematics', 'noiseIdentification', 'plotters', 'statistics'};
for i = 1:length(subfolders)
    addpath(fullfile(tool_path, subfolders{i}))
end
fprintf('\n Matlab tool folders added to the path \n\n');
