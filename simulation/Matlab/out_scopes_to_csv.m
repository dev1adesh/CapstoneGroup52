%% out_scopes_to_csv.m
% Read the "out" workspace variable from Simulink and export each scope
% group to a CSV for debugging.
%
% Expected structure: out contains (or out is a struct with):
%   AngularOrientation  - Angular Roll, Pitch, Yaw (3 signals)
%   PositionOutput      - Wheel position (3 signals)
%   RollPitchYaw        - Platform orientation (3 signals)
%   RollTorPitchTorYawTor - Roll/Pitch/Yaw torque (3 signals)
%   TorqueInput         - T1, T2, T3 (3 signals)
%
% Usage:
%   After simulation: out_scopes_to_csv()
%   Or after load('file.mat'): out_scopes_to_csv()
%   Or pass variable name: out_scopes_to_csv('out')
%
% CSVs are written to the current directory: AngularOrientation.csv, etc.

function out_scopes_to_csv(varName)
    if nargin < 1
        varName = 'out';
    end

    if ~evalin('base', ['exist(''' varName ''', ''var'')'])
        error('Variable "%s" not in base workspace. Run simulation or load a .mat first.', varName);
    end
    out = evalin('base', varName);

    % If out is Simulink.SimulationOutput, find the Dataset (logsout, yout, or any Dataset property)
    if isa(out, 'Simulink.SimulationOutput')
        data = get_dataset_from_simulation_output(out);
        if ~isempty(data)
            out = data;
        end
    end

    % Build list of possible names per scope (exact + common variants)
    scopeDefs = {
        'AngularOrientation',   {'time', 'roll_deg', 'pitch_deg', 'yaw_deg'},   {'AngularOrientation','Angular Orientation'};
        'PositionOutput',       {'time', 'wheel1_pos', 'wheel2_pos', 'wheel3_pos'}, {'PositionOutput','Position Output'};
        'RollPitchYaw',         {'time', 'roll_deg', 'pitch_deg', 'yaw_deg'},   {'RollPitchYaw','Roll Pitch Yaw'};
        'RollTorPitchTorYawTor', {'time', 'roll_torque', 'pitch_torque', 'yaw_torque'}, {'RollTorPitchTorYawTor','Roll Tor Pitch Tor Yaw Tor'};
        'TorqueInput',          {'time', 'T1', 'T2', 'T3'},                     {'TorqueInput','Torque Input'};
    };

    wroteAny = false;
    for i = 1:size(scopeDefs, 1)
        scopeName = scopeDefs{i, 1};
        colNames  = scopeDefs{i, 2};
        altNames  = scopeDefs{i, 3};
        [t, D] = get_scope_three_signals(out, scopeName, altNames);
        if isempty(t)
            continue;
        end
        csvPath = [scopeName '.csv'];
        write_scope_csv(csvPath, t, D, colNames);
        fprintf('Wrote %s\n', csvPath);
        wroteAny = true;
    end
    if ~wroteAny
        fprintf('No scope data found. Structure of "%s":\n', varName);
        print_out_structure(out);
    end
end

function [t, D] = get_scope_three_signals(out, scopeName, altNames)
    if nargin < 3
        altNames = {scopeName};
    end
    t = [];
    D = [];
    % Try Dataset first (Simulink default)
    if isa(out, 'Simulink.SimulationData.Dataset')
        [t, D] = get_from_dataset(out, scopeName, altNames);
        return;
    end
    % Struct: try exact and alternate names
    S = [];
    for n = 1:numel(altNames)
        S = get_scope_struct(out, altNames{n});
        if ~isempty(S)
            break;
        end
    end
    if isempty(S)
        [t, D] = get_scope_by_index(out, scopeName);
        return;
    end
    [t, D] = unpack_time_signals(S);
end

function [t, D] = get_from_dataset(out, scopeName, altNames)
    t = [];
    D = [];
    nEl = out.numElements;
    % Keywords for fuzzy match if exact/alt names fail
    keyMap = {'AngularOrientation', 'Angular'; 'PositionOutput', 'Position'; ...
              'RollPitchYaw', 'RollPitchYaw'; 'RollTorPitchTorYawTor', 'Tor'; 'TorqueInput', 'Torque'};
    for i = 1:nEl
        el = out.getElement(i);
        name = '';
        if isprop(el, 'Name'), name = el.Name; end
        if isempty(name) && isstruct(el) && isfield(el, 'Name'), name = el.Name; end
        name = strtrim(char(name));
        nameNorm = strrep(lower(name), ' ', '');
        match = false;
        for n = 1:numel(altNames)
            if strcmp(nameNorm, strrep(lower(altNames{n}), ' ', ''))
                match = true;
                break;
            end
        end
        if ~match
            for km = 1:size(keyMap, 1)
                if strcmp(keyMap{km, 1}, scopeName) && contains(nameNorm, lower(keyMap{km, 2}))
                    match = true;
                    break;
                end
            end
        end
        if ~match, continue; end
        % Get time and data from element
        if isprop(el, 'Values')
            V = el.Values;
        elseif isstruct(el) && isfield(el, 'Values')
            V = el.Values;
        else
            continue;
        end
        if isa(V, 'timeseries')
            t = V.Time(:);
            dat = V.Data;
            if numel(size(dat)) == 2 && size(dat, 2) >= 3
                D = dat(:, 1:3);
            elseif isvector(dat)
                D = dat(:);
            else
                D = dat;
            end
            if size(D, 2) < 3 && size(D, 1) > 0
                D = [D, zeros(numel(t), 3 - size(D, 2))];
            end
            return;
        end
        if isstruct(V) && isfield(V, 'time')
            [t, D] = unpack_time_signals(V);
            return;
        end
    end
end

function [t, D] = unpack_time_signals(S)
    t = [];
    D = [];
    if ~isstruct(S) || ~isfield(S, 'time')
        return;
    end
    t = S.time(:);
    if isfield(S, 'signals')
        sigs = S.signals;
        nSig = numel(sigs);
        nT = numel(t);
        D = zeros(nT, 3);
        for j = 1:min(3, nSig)
            s = sigs(j);
            if isfield(s, 'values')
                V = s.values;
            elseif isfield(s, 'value')
                V = s.value;
            else
                continue;
            end
            if isvector(V)
                D(:, j) = V(:);
            else
                D(:, j) = V(:, 1);
            end
        end
        if nSig == 1 && (isfield(sigs, 'values') || isfield(sigs, 'value'))
            V = sigs(1).values;
            if size(V, 2) >= 3
                D = V(:, 1:3);
            end
        end
    elseif isfield(S, 'values')
        V = S.values;
        if size(V, 2) >= 3
            D = V(:, 1:3);
        end
    end
end

function [t, D] = get_scope_by_index(out, scopeName)
    % When out has .time and .signals only (no named sub-structs), use fixed order.
    t = [];
    D = [];
    if ~isstruct(out) || ~isfield(out, 'time') || ~isfield(out, 'signals')
        return;
    end
    t = out.time(:);
    sigs = out.signals;
    idx = scope_name_to_index(scopeName);
    if isempty(idx) || idx(1) + 2 > numel(sigs)
        return;
    end
    nT = numel(t);
    D = zeros(nT, 3);
    for j = 1:3
        s = sigs(idx(1) + j - 1);
        if isfield(s, 'values')
            V = s.values;
        elseif isfield(s, 'value')
            V = s.value;
        else
            continue;
        end
        if isvector(V)
            D(:, j) = V(:);
        else
            D(:, j) = V(:, 1);
        end
    end
end

function idx = scope_name_to_index(scopeName)
    % Order in one big .signals: AngularOrientation 1-3, PositionOutput 4-6, RollPitchYaw 7-9, RollTor.. 10-12, TorqueInput 13-15
    map = {'AngularOrientation', 1; 'PositionOutput', 4; 'RollPitchYaw', 7; 'RollTorPitchTorYawTor', 10; 'TorqueInput', 13};
    idx = [];
    for k = 1:size(map, 1)
        if strcmp(map{k, 1}, scopeName)
            idx = map{k, 2};
            return;
        end
    end
end

function S = get_scope_struct(out, scopeName)
    S = [];
    if isstruct(out) && isfield(out, scopeName)
        S = out.(scopeName);
        return;
    end
    % Simulink.SimulationOutput (or any object) with scope as property: out.AngularOrientation, etc.
    if isobject(out) && isprop(out, scopeName)
        try
            S = out.(scopeName);
        catch %#ok<CTCH>
        end
        if ~isempty(S)
            return;
        end
    end
    % Simulink.SimulationData.Dataset
    if isa(out, 'Simulink.SimulationData.Dataset')
        try
            el = out.get(scopeName);
            if ~isempty(el)
                S = el.Values;
            end
        catch %#ok<CTCH>
        end
        return;
    end
    % Try nested: e.g. out.scope1.AngularOrientation
    if isstruct(out)
        fns = fieldnames(out);
        for k = 1:numel(fns)
            sub = out.(fns{k});
            if isstruct(sub) && isfield(sub, scopeName)
                S = sub.(scopeName);
                return;
            end
        end
    end
end

function write_scope_csv(csvPath, t, D, colNames)
    ncol = 1 + size(D, 2);
    if numel(colNames) < ncol
        colNames = [colNames, arrayfun(@(c) sprintf('col_%d', c), numel(colNames)+1:ncol, 'UniformOutput', false)];
    end
    colNames = colNames(1:ncol);
    T = array2table([t, D], 'VariableNames', colNames);
    writetable(T, csvPath);
end

function data = get_dataset_from_simulation_output(out)
    % Use who() to get all property names, then find first that is a Dataset
    data = [];
    try
        props = who(out);
    catch
        props = {};
    end
    if isempty(props)
        % Fallback: try known names
        props = {'logsout','yout','recordout','simout','ScopeData','out'};
    end
    for k = 1:numel(props)
        pname = props{k};
        if ischar(pname), pname = char(pname); end
        try
            v = out.(pname);
            if isa(v, 'Simulink.SimulationData.Dataset') && v.numElements > 0
                data = v;
                return;
            end
        catch %#ok<CTCH>
        end
    end
    % If who didn't return anything useful, try get(out, name) for known names
    for pname = {'logsout','yout','recordout','simout'}
        try
            v = get(out, pname{1});
            if isa(v, 'Simulink.SimulationData.Dataset') && v.numElements > 0
                data = v;
                return;
            end
        catch %#ok<CTCH>
        end
    end
end

function print_out_structure(out)
    if isa(out, 'Simulink.SimulationOutput')
        fprintf('  Type: Simulink.SimulationOutput.\n');
        try
            props = who(out);
            fprintf('  Properties (who): %s\n', strjoin(cellstr(props), ', '));
            for k = 1:numel(props)
                pname = props{k};
                try
                    v = out.(pname);
                    fprintf('  .%s: %s', pname, class(v));
                    if isa(v, 'Simulink.SimulationData.Dataset')
                        fprintf('  numElements = %d', v.numElements);
                        for i = 1:min(v.numElements, 10)
                            el = v.getElement(i);
                            nm = '';
                            if isprop(el, 'Name'), nm = el.Name; end
                            fprintf('\n    [%d] "%s"', i, strtrim(char(nm)));
                        end
                        if v.numElements > 10
                            fprintf('\n    ... and %d more', v.numElements - 10);
                        end
                    end
                    fprintf('\n');
                catch e
                    fprintf('  .%s: (error: %s)\n', pname, e.message);
                end
            end
        catch e
            fprintf('  (who failed: %s)\n', e.message);
        end
        return;
    end
    if isa(out, 'Simulink.SimulationData.Dataset')
        fprintf('  Type: Simulink.SimulationData.Dataset, numElements = %d\n', out.numElements);
        for i = 1:min(out.numElements, 20)
            el = out.getElement(i);
            name = '';
            if isprop(el, 'Name'), name = el.Name; end
            if isstruct(el) && isfield(el, 'Name'), name = el.Name; end
            fprintf('  Element %d: Name = "%s"\n', i, strtrim(char(name)));
        end
        if out.numElements > 20
            fprintf('  ... and %d more\n', out.numElements - 20);
        end
        return;
    end
    if isstruct(out)
        fns = fieldnames(out);
        fprintf('  Type: struct, fields (%d): %s\n', numel(fns), strjoin(fns, ', '));
        for k = 1:min(numel(fns), 15)
            v = out.(fns{k});
            if isstruct(v)
                sub = fieldnames(v);
                fprintf('  .%s: struct with %s\n', fns{k}, strjoin(sub, ', '));
            else
                fprintf('  .%s: %s\n', fns{k}, class(v));
            end
        end
        return;
    end
    fprintf('  Type: %s\n', class(out));
end
