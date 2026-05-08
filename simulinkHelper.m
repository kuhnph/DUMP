classdef SimModelBuilder < handle
    % SimModelBuilder
    %
    % Reusable helper class for programmatically creating Simulink models.
    %
    % Primary current use:
    %   - Create lookup-table models from a struct in the MATLAB base workspace
    %   - One input breakpoint signal
    %   - One 1-D Lookup Table per output column
    %   - Mux multi-column outputs into vector outports
    %
    % Example:
    %   builder = SimModelBuilder(cfg);
    %   builder.validateWorkspaceStruct();
    %   builder.assignLookupVariables();
    %   builder.openOrCreateModel();
    %   builder.cleanGeneratedContent();
    %   builder.addInput(cfg.breakpointField);
    %   builder.addLookupOutputs();
    %   builder.finalizeModel();

    properties
        cfg
        modelName
        S
        breakpoint
        layout
        verbose = true
    end

    methods
        function obj = SimModelBuilder(cfg)
            arguments
                cfg struct
            end

            obj.cfg = cfg;
            obj.modelName = cfg.modelName;

            if isfield(cfg, 'verbose')
                obj.verbose = cfg.verbose;
            end

            obj.layout = obj.defaultLayout();
        end

        function validateWorkspaceStruct(obj)
            % Validate:
            %   - struct exists in base workspace
            %   - breakpoint field exists and is vector
            %   - lookup fields exist
            %   - lookup fields have same number of rows as breakpoint

            structName = obj.cfg.structName;
            breakpointField = obj.cfg.breakpointField;
            lookupFields = obj.cfg.lookupFields;

            structExists = evalin('base', sprintf('exist(''%s'', ''var'')', structName));

            if ~structExists
                error('Struct "%s" does not exist in the base workspace.', structName);
            end

            S_local = evalin('base', structName);

            if ~isstruct(S_local)
                error('"%s" exists, but it is not a struct.', structName);
            end

            if ~isfield(S_local, breakpointField)
                error('Struct "%s" does not contain breakpoint field "%s".', ...
                    structName, breakpointField);
            end

            bp = S_local.(breakpointField);

            if ~isvector(bp)
                error('Breakpoint field "%s" must be a vector.', breakpointField);
            end

            bp = bp(:);

            for k = 1:numel(lookupFields)
                fieldName = lookupFields{k};

                if strcmp(fieldName, breakpointField)
                    error(['lookupFields contains "%s", but this is the breakpoint/input. ', ...
                           'Remove it from lookupFields.'], breakpointField);
                end

                if ~isfield(S_local, fieldName)
                    error('Struct "%s" does not contain lookup field "%s".', ...
                        structName, fieldName);
                end

                tableData = S_local.(fieldName);

                if isvector(tableData)
                    tableData = tableData(:);
                end

                if size(tableData, 1) ~= numel(bp)
                    error('Field "%s" has %d rows, but %s has %d elements.', ...
                        fieldName, size(tableData, 1), breakpointField, numel(bp));
                end
            end

            obj.S = S_local;
            obj.breakpoint = bp;

            obj.log('Workspace data validation passed.');
        end

        function assignLookupVariables(obj)
            % Assign:
            %   breakpoint variable:
            %       TALO_HighRate_bp
            %
            %   table variables:
            %       Vehicle0_X_ECEF_HighRate_col1_table
            %       Vehicle0_X_ECEF_HighRate_col2_table
            %       ...

            breakpointVarName = obj.cfg.breakpointVarName;
            lookupFields = obj.cfg.lookupFields;

            assignin('base', breakpointVarName, obj.breakpoint);
            obj.log('Assigned breakpoint variable: %s', breakpointVarName);

            for k = 1:numel(lookupFields)
                fieldName = lookupFields{k};

                tableData = obj.S.(fieldName);

                if isvector(tableData)
                    tableData = tableData(:);
                end

                nCols = size(tableData, 2);

                for c = 1:nCols
                    tableVarName = obj.tableVarName(fieldName, c);
                    assignin('base', tableVarName, tableData(:, c));

                    obj.log('Assigned table variable: %s', tableVarName);
                end
            end
        end

        function openOrCreateModel(obj)
            % Create or open model.

            modelName = obj.modelName;

            try
                if bdIsLoaded(modelName)
                    obj.log('Model "%s" is already loaded. Reusing it.', modelName);

                elseif exist([modelName '.slx'], 'file')
                    obj.log('Model file "%s.slx" exists. Loading it.', modelName);
                    load_system(modelName);

                else
                    obj.log('Creating new model "%s".', modelName);
                    new_system(modelName);
                end

                open_system(modelName);

            catch ME
                error('Could not create/open model "%s": %s', modelName, ME.message);
            end
        end

        function cleanGeneratedContent(obj)
            % Delete generated lines, LUTs, Muxes, and optionally outports.

            if ~isfield(obj.cfg, 'cleanGeneratedBlocks') || ~obj.cfg.cleanGeneratedBlocks
                return;
            end

            modelName = obj.modelName;

            obj.deleteAllLines();

            try
                topBlocks = find_system(modelName, ...
                    'SearchDepth', 1, ...
                    'Type', 'Block');

                for i = 1:numel(topBlocks)
                    blk = topBlocks{i};
                    [~, blkName] = fileparts(blk);

                    isLUT = startsWith(blkName, 'LUT_');
                    isMux = startsWith(blkName, 'Mux_');

                    isOutportToClean = false;
                    if isfield(obj.cfg, 'cleanOutports') && obj.cfg.cleanOutports
                        isOutportToClean = any(strcmp(blkName, obj.cfg.lookupFields));
                    end

                    if isLUT || isMux || isOutportToClean
                        obj.safeDeleteBlock(blk);
                    end
                end

            catch ME
                warning('Could not clean generated blocks: %s', ME.message);
            end
        end

        function addInput(obj, inputName)
            % Add or reuse input block.

            L = obj.layout;

            inportPath = [obj.modelName '/' inputName];

            pos = [
                L.x0, ...
                L.y0, ...
                L.x0 + L.inputWidth, ...
                L.y0 + L.inputHeight
            ];

            obj.addOrReuseBlock('simulink/Sources/In1', inportPath, pos);
        end

        function addLookupOutputs(obj)
            % Create all lookup outputs listed in cfg.lookupFields.

            lookupFields = obj.cfg.lookupFields;

            for k = 1:numel(lookupFields)
                fieldName = lookupFields{k};

                tableData = obj.S.(fieldName);

                if isvector(tableData)
                    tableData = tableData(:);
                end

                nCols = size(tableData, 2);

                obj.createLookupVectorOutput(fieldName, nCols, k);
            end
        end

        function finalizeModel(obj)
            % Set model-level params, arrange, and save.

            modelName = obj.modelName;

            if isfield(obj.cfg, 'stopTime')
                stopTime = obj.cfg.stopTime;
            else
                stopTime = '10';
            end

            try
                set_param(modelName, ...
                    'StopTime', stopTime, ...
                    'Solver', 'FixedStepAuto');

                obj.log('Model parameters updated.');

            catch ME
                warning('Could not set model parameters: %s', ME.message);
            end

            try
                Simulink.BlockDiagram.arrangeSystem(modelName);
                obj.log('Arranged model layout.');
            catch ME
                warning('Could not auto-arrange model: %s', ME.message);
            end

            try
                save_system(modelName);
                obj.log('Created/updated and saved model: %s.slx', modelName);
            catch ME
                warning('Could not save model "%s": %s', modelName, ME.message);
            end
        end
    end

    methods (Access = private)

        function createLookupVectorOutput(obj, fieldName, nCols, rowIndex)
            % Create:
            %
            %   input -> LUT_col1 --\
            %   input -> LUT_col2 ----> Mux -> Outport
            %   input -> LUT_col3 --/
            %
            % Or:
            %
            %   input -> LUT_col1 -> Outport

            modelName = obj.modelName;
            inputName = obj.cfg.breakpointField;
            breakpointVarName = obj.cfg.breakpointVarName;
            L = obj.layout;

            baseY = L.y0 + (rowIndex - 1) * L.rowSpacing;

            muxName = ['Mux_' fieldName];
            muxPath = [modelName '/' muxName];

            outName = fieldName;
            outPath = [modelName '/' outName];

            % Add/reuse outport
            outPos = [
                L.outX, ...
                baseY, ...
                L.outX + L.outportWidth, ...
                baseY + L.outportHeight
            ];

            obj.addOrReuseBlock('simulink/Sinks/Out1', outPath, outPos);

            % Add/reuse mux if multi-column
            if nCols > 1
                muxHeight = max(60, 20 * nCols);

                muxPos = [
                    L.muxX, ...
                    baseY - 20, ...
                    L.muxX + L.muxWidth, ...
                    baseY - 20 + muxHeight
                ];

                obj.addOrReuseBlock('simulink/Signal Routing/Mux', muxPath, muxPos);
                obj.safeSetParam(muxPath, 'Inputs', num2str(nCols));
            end

            % Add one LUT per column
            for c = 1:nCols
                lutName = sprintf('LUT_%s_col%d', fieldName, c);
                lutPath = [modelName '/' lutName];

                y = baseY + (c - 1) * L.colSpacing;

                lutPos = [
                    L.lutX, ...
                    y - 10, ...
                    L.lutX + L.lutWidth, ...
                    y - 10 + L.lutHeight
                ];

                obj.addOrReuseBlock('simulink/Lookup Tables/1-D Lookup Table', lutPath, lutPos);

                tableVarName = obj.tableVarName(fieldName, c);

                obj.safeSetParam(lutPath, ...
                    'BreakpointsForDimension1', breakpointVarName, ...
                    'Table', tableVarName, ...
                    'InterpMethod', 'Linear point-slope', ...
                    'ExtrapMethod', 'Linear');

                obj.safeAddLine(modelName, [inputName '/1'], [lutName '/1']);

                if nCols > 1
                    obj.safeAddLine(modelName, [lutName '/1'], sprintf('%s/%d', muxName, c));
                else
                    obj.safeAddLine(modelName, [lutName '/1'], [outName '/1']);
                end
            end

            if nCols > 1
                obj.safeAddLine(modelName, [muxName '/1'], [outName '/1']);
            end
        end

        function tf = blockExists(~, blockPath)
            try
                get_param(blockPath, 'Handle');
                tf = true;
            catch
                tf = false;
            end
        end

        function addOrReuseBlock(obj, sourceBlock, targetPath, position)
            try
                if obj.blockExists(targetPath)
                    obj.log('Block already exists: %s', targetPath);
                    set_param(targetPath, 'Position', position);
                else
                    add_block(sourceBlock, targetPath, 'Position', position);
                    obj.log('Added block: %s', targetPath);
                end

            catch ME
                warning('Could not add/reuse block "%s": %s', targetPath, ME.message);
            end
        end

        function ok = safeSetParam(~, blockPath, varargin)
            ok = true;

            try
                set_param(blockPath, varargin{:});
            catch ME
                ok = false;
                warning('Could not set parameters for "%s": %s', ...
                    blockPath, ME.message);
            end
        end

        function ok = safeAddLine(~, modelName, src, dst)
            ok = true;

            try
                add_line(modelName, src, dst, 'autorouting', 'on');
            catch ME
                ok = false;
                warning('Could not connect "%s" to "%s": %s', ...
                    src, dst, ME.message);
            end
        end

        function safeDeleteBlock(obj, blockPath)
            try
                delete_block(blockPath);
                obj.log('Deleted generated block: %s', blockPath);
            catch ME
                warning('Could not delete block "%s": %s', blockPath, ME.message);
            end
        end

        function deleteAllLines(obj)
            modelName = obj.modelName;

            try
                lines = find_system(modelName, ...
                    'FindAll', 'on', ...
                    'Type', 'line');

                for i = 1:numel(lines)
                    try
                        delete_line(lines(i));
                    catch
                    end
                end

                obj.log('Deleted existing lines.');

            catch ME
                warning('Could not delete existing lines: %s', ME.message);
            end
        end

        function name = tableVarName(~, fieldName, colIndex)
            name = sprintf('%s_col%d_table', fieldName, colIndex);
        end

        function L = defaultLayout(~)
            L = struct();

            L.x0 = 100;
            L.y0 = 100;

            L.inputWidth = 110;
            L.inputHeight = 20;

            L.lutX = L.x0 + 240;
            L.lutWidth = 210;
            L.lutHeight = 35;

            L.muxX = L.x0 + 540;
            L.muxWidth = 35;

            L.outX = L.x0 + 680;
            L.outportWidth = 220;
            L.outportHeight = 20;

            L.rowSpacing = 150;
            L.colSpacing = 45;
        end

        function log(obj, varargin)
            if obj.verbose
                fprintf([varargin{1} '\n'], varargin{2:end});
            end
        end
    end
end
