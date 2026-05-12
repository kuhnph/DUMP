function buildSubsystemWrapper(obj, sourceModelName, wrapperModelName, subsystemName, busOutportName)
    % buildSubsystemWrapper
    %
    % Creates a new wrapper model around an existing Simulink model.
    %
    % This version copies the contents of sourceModelName directly into
    % one subsystem. It does NOT use a Model Reference block.
    %
    % Result:
    %
    %   wrapper model
    %       |
    %       +--> top-level inports
    %       |
    %       +--> subsystem containing copied source model contents
    %       |
    %       +--> individual top-level outports
    %       |
    %       +--> Bus Creator --> bus outport

    if nargin < 5
        busOutportName = 'OutputBus';
    end

    %% Load source model

    if ~bdIsLoaded(sourceModelName)
        load_system(sourceModelName);
    end

    %% Read top-level inports/outports from source model

    srcInports = find_system(sourceModelName, ...
        'SearchDepth', 1, ...
        'BlockType', 'Inport');

    srcOutports = find_system(sourceModelName, ...
        'SearchDepth', 1, ...
        'BlockType', 'Outport');

    if isempty(srcInports)
        error('Source model "%s" has no top-level Inport blocks.', sourceModelName);
    end

    if isempty(srcOutports)
        error('Source model "%s" has no top-level Outport blocks.', sourceModelName);
    end

    srcInports = obj.sortPortsByPortNumber(srcInports);
    srcOutports = obj.sortPortsByPortNumber(srcOutports);

    inputNames = obj.getBlockNames(srcInports);
    outputNames = obj.getBlockNames(srcOutports);

    nIn = numel(inputNames);
    nOut = numel(outputNames);

    obj.log('Source model "%s" has %d inport(s) and %d outport(s).', ...
        sourceModelName, nIn, nOut);

    %% Create or reset wrapper model

    if bdIsLoaded(wrapperModelName)
        close_system(wrapperModelName, 0);
    end

    if exist([wrapperModelName '.slx'], 'file') == 2
        delete([wrapperModelName '.slx']);
    end

    new_system(wrapperModelName);
    open_system(wrapperModelName);

    %% Layout settings

    xIn = 80;
    xSub = 300;
    xOut = 750;
    xBus = 750;
    xBusOut = 950;

    y0 = 100;
    dy = 70;

    subWidth = 300;
    subHeight = max(160, max(nIn, nOut) * dy);

    %% Add one subsystem to wrapper model

    subsystemPath = [wrapperModelName '/' subsystemName];

    add_block('simulink/Ports & Subsystems/Subsystem', subsystemPath, ...
        'Position', [xSub y0 xSub + subWidth y0 + subHeight]);

    obj.clearSubsystemContents(subsystemPath);

    %% Copy source model contents directly into subsystem
    %
    % This is the key change.
    % Instead of:
    %   subsystem -> Model Reference -> source model
    %
    % We now do:
    %   subsystem -> copied blocks/lines from source model

    try
        Simulink.BlockDiagram.copyContentsToSubSystem(sourceModelName, subsystemPath);
        obj.log('Copied contents of "%s" into subsystem "%s".', ...
            sourceModelName, subsystemPath);
    catch ME
        error('Could not copy contents of "%s" into subsystem "%s": %s', ...
            sourceModelName, subsystemPath, ME.message);
    end

    %% Add wrapper top-level inports and connect to subsystem

    for i = 1:nIn
        inName = inputNames{i};

        wrapperInPath = [wrapperModelName '/' inName];

        add_block('simulink/Sources/In1', wrapperInPath, ...
            'Position', [xIn y0 + (i-1)*dy xIn + 80 y0 + 20 + (i-1)*dy]);

        obj.safeAddLine(wrapperModelName, ...
            sprintf('%s/1', inName), ...
            sprintf('%s/%d', subsystemName, i));
    end

    %% Add individual wrapper outports

    for i = 1:nOut
        outName = outputNames{i};

        wrapperOutPath = [wrapperModelName '/' outName];

        add_block('simulink/Sinks/Out1', wrapperOutPath, ...
            'Position', [xOut y0 + (i-1)*dy xOut + 180 y0 + 20 + (i-1)*dy]);

        obj.safeAddLine(wrapperModelName, ...
            sprintf('%s/%d', subsystemName, i), ...
            sprintf('%s/1', outName));
    end

    %% Add Bus Creator and bus outport

    busCreatorName = 'BusCreator';
    busCreatorPath = [wrapperModelName '/' busCreatorName];
    busOutPath = [wrapperModelName '/' busOutportName];

    busTop = y0 + nOut*dy + 80;
    busHeight = max(80, nOut * 20);

    add_block('simulink/Signal Routing/Bus Creator', busCreatorPath, ...
        'Inputs', num2str(nOut), ...
        'Position', [xBus busTop xBus + 80 busTop + busHeight]);

    add_block('simulink/Sinks/Out1', busOutPath, ...
        'Position', [xBusOut busTop + busHeight/2 - 10 ...
                     xBusOut + 120 busTop + busHeight/2 + 10]);

    for i = 1:nOut
        obj.safeAddLine(wrapperModelName, ...
            sprintf('%s/%d', subsystemName, i), ...
            sprintf('%s/%d', busCreatorName, i));
    end

    obj.safeAddLine(wrapperModelName, ...
        sprintf('%s/1', busCreatorName), ...
        sprintf('%s/1', busOutportName));

    %% Finalize

    try
        Simulink.BlockDiagram.arrangeSystem(wrapperModelName);
    catch ME
        warning('Could not auto-arrange wrapper model: %s', ME.message);
    end

    save_system(wrapperModelName);

    obj.log('Created wrapper model "%s.slx" with source contents copied into subsystem "%s".', ...
        wrapperModelName, subsystemName);
end
