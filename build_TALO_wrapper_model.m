%% build_TALO_wrapper_model.m
%
% Wrap an existing lookup-table model in a new model.
%
% Source model:
%   TALO_Lookup_Model
%
% New wrapper model:
%   TALO_Wrapper_Model
%
% Result:
%   - wrapper inport(s)
%   - subsystem containing the source lookup model
%   - individual outports
%   - bus creator containing all subsystem outputs
%   - bus outport

clear
clc

sourceModelName = 'TALO_Lookup_Model';
wrapperModelName = 'TALO_Wrapper_Model';
subsystemName = 'TALO_Lookup_Subsystem';
busOutportName = 'TALO_Output_Bus';

cfg = struct();
cfg.modelName = wrapperModelName;
cfg.verbose = true;

builder = SimModelBuilder(cfg);

builder.buildSubsystemWrapper( ...
    sourceModelName, ...
    wrapperModelName, ...
    subsystemName, ...
    busOutportName);
