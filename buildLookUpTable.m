%% build_TALO_HighRate_lookup_model.m
%
% High-level model generation script.
%
% Assumes:
%   - myData already exists in the MATLAB base workspace
%   - SimModelBuilder.m is on the MATLAB path or in the current folder

clearvars -except myData

cfg = struct();

%% =========================
%  MODEL SETTINGS
%  =========================

cfg.modelName = 'Talo_HighRate_Lookup_Model';

% Name of the struct already loaded in the base workspace
cfg.structName = 'myData';

% Breakpoint/input field inside the struct
cfg.breakpointField = 'TALO_HighRate';

% Name of breakpoint variable to create in the base workspace
cfg.breakpointVarName = 'TALO_HighRate_bp';

% Fields to output from the lookup model.
% IMPORTANT:
% Do NOT include TALO_HighRate here.
cfg.lookupFields = {
    'Vehicle0_X_ECEF_HighRate'
    'Vehicle0_V_ECEF_HighRate'
    'Vehicle0_A_ECEF_HighRate'
    'Vehicle0_wbf_HighRate'
    'Vehicle0_Euler321_YawPitRoll_HighRate'
    'Vehicle0_LatLongAlt_HighRate'
    'Vehicle0_Vb_HighRate'
    'Vehicle0_Force_HighRate'
    'Vehicle0_Moment_HighRate'
    'Vehicle0_q_bi_HighRate'
    'Aerodynamics_Mach'
    'Aerodynamics_AirSpeed'
};

% Delete old LUTs, Muxes, and lines before rebuilding
cfg.cleanGeneratedBlocks = true;

% Usually false is safer.
% Set true only if you want old generated outports deleted too.
cfg.cleanOutports = false;

cfg.stopTime = '10';

cfg.verbose = true;

%% =========================
%  BUILD MODEL
%  =========================

builder = SimModelBuilder(cfg);

builder.validateWorkspaceStruct();
builder.assignLookupVariables();
builder.openOrCreateModel();
builder.cleanGeneratedContent();
builder.addInput(cfg.breakpointField);
builder.addLookupOutputs();
builder.finalizeModel();
