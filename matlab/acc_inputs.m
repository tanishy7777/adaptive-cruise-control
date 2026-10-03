function [p, scenarios, root] = acc_inputs()
%ACC_INPUTS Shared versioned configuration used by all three implementations.
root = fileparts(fileparts(mfilename('fullpath')));
p = jsondecode(fileread(fullfile(root,'config','parameters.json')));
scenarios = jsondecode(fileread(fullfile(root,'config','scenarios.json')));
end
