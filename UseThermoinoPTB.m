% Interface script for MEDOC thermodes, for use with arduino software v3.5 or higher.
% [varargout] = UseThermoinoPTB(action,varargin)
% ========
% THERMODE
% ========
% UseThermoinoPTB('init',ComPort, BaselineTemp, RateOfRise)
% creates a global struct array thermoino
% all functions that use UseThermoinoPTB should declare this as global (global thermoino;)
%
% UseThermoinoPTB('id',Index) --> returns the name of thermoino Index
% UseThermoinoPTB('id') --> returns the name of 1st thermoino
%
% UseThermoinoPTB('time',Index) --> returns the time (ms) of thermoino Index
% UseThermoinoPTB('time') --> returns the time (ms) of 1st thermoino
%
% UseThermoinoPTB('trigger',Index) --> START thermoino Index
% UseThermoinoPTB('trigger') --> START 1st thermoino
% returns PTB time of start (or error string e.g. if busy)
%
% UseThermoinoPTB('move',dur,Index) --> MOVE (dur in 탎) thermoino Index
% UseThermoinoPTB('move',dur) --> MOVE (dur in 탎) 1st thermoino
% returns PTB time of start (or error string e.g. if busy)
%
% UseThermoinoPTB('set',TargetTemp,Index) --> MOVE thermoino Index to TargetTemp in 캜
% UseThermoinoPTB('set',TargetTemp) --> MOVE 1st thermoino to TargetTemp in 캜
% returns PTB time of start (or error string e.g. if busy)
%
% UseThermoinoPTB('loadctc',period,data,Index) --> uploads data to thermoino Index and sets period (all in ms)
% UseThermoinoPTB('loadctc',period,data)  --> uploads data to 1st thermoino and sets period (all in ms)
%
% UseThermoinoPTB('queryctc',Index) --> read CTC data from thermoino Index
% UseThermoinoPTB('queryctc') --> read CTC data from 1st thermoino
% returns CTC data as a n x 2 matrix column 1: 0..n-1, column 2: data
%
% UseThermoinoPTB('execctc',Index) --> start CTC on thermoino Index
% UseThermoinoPTB('execctc')  --> start CTC on 1st thermoino Index
% returns PTB time of start (or error string e.g. if busy)
%
% UseThermoinoPTB('flushctc',Index) --> clear CTC data in thermoino Index
% UseThermoinoPTB('flushctc')  --> clear CTC data in 1st thermoino
% returns PTB time of start (or error string e.g. if busy)%%
%
%
% =========
% DIGITIMER
% =========
% UseThermoinoPTB('shock',nShocks,ISI, Index) --> apply nShocks with ISI (thermoino Index)
% UseThermoinoPTB('shock',nShocks,ISI) --> apply nShocks with ISI (1st thermoino)
% returns PTB time of start (or error string e.g. if busy)

function [varargout] = UseThermoinoPTB(action,varargin)

current_version = 3.50;
min_isi         = 1100; % 1100탎 minimum shock ISI
baudrate        = 115200;
maxperiod       = 500;
maxctc          = 2500;
global thermoino; % make sure this is accessible everywhere

% define some constants

ERRORMSG = {}; % errors as defined in Thermode_PWM.ino
ERRORMSG{end+1} = 'ERR_NO_PARAM';
ERRORMSG{end+1} = 'ERR_CMD_NOT_FOUND';
ERRORMSG{end+1} = 'ERR_CTC_BIN_WIDTH';
ERRORMSG{end+1} = 'ERR_CTC_PULSE_WIDTH';
ERRORMSG{end+1} = 'ERR_CTC_NOT_INIT';
ERRORMSG{end+1} = 'ERR_CTC_FULL';
ERRORMSG{end+1} = 'ERR_CTC_EMPTY';
ERRORMSG{end+1} = 'ERR_SHOCK_RANGE';
ERRORMSG{end+1} = 'ERR_SHOCK_ISI';
ERRORMSG{end+1} = 'ERR_BUSY';
ERRORMSG{end+1} = 'ERR_DEBUG_RANGE';
ERRORMSG{end+1} = 'ERR_CHANNEL_RANGE';

switch lower(action)
    
    %-----------------------------------------------------------
    case 'init' % UseThermoino('init',ComPort, BaselineTemp, RateOfRise)
        % creates a global struct array thermoino
        % all functions that use UseThermoino should declare this
        % as global (global thermoino;)
        %-----------------------------------------------------------
        oldverbosity = IOPort('Verbosity', 0);
        [h, errmsg] = IOPort('OpenSerialPort', varargin{1},'BaudRate=1200,DTR=1,RTS=1');
        if h >= 0
            IOPort('Close', h); %OK Port exists and can be opened *** At the same time this resets the Leonardo
        elseif isempty(strfind(errmsg, 'ENOENT'))
            error([varargin{1} ' is already open']);
        else
            error([varargin{1} ' does not exist']);
        end
        IOPort('Verbosity', oldverbosity);
        s  = IOPort('OpenSerialPort', char(varargin{1}),'BaudRate=115200,DTR=1,RTS=1');
        WaitSecs(1);
        IOPort('Purge',s);
        IOPort('Flush',s);
        
        ver = mywriteread(s,"VER",6);
        switch true
            case  str2num(ver) >= current_version % check for latest thermoino
                ind = numel(thermoino)+1;
                thermoino(ind).port   = varargin{1};
                thermoino(ind).id     = mywriteread(s,"GETID");
                thermoino(ind).handle = s;
                thermoino(ind).t      = varargin{2};
                thermoino(ind).t_init = varargin{2};
                thermoino(ind).ror    = varargin{3};
                thermoino(ind).ctc    = [];
                thermoino(ind).period = [];
                clear s;
            case str2num(ver) == -2
                IOPort('Close',s);
                error(['Command VER not implemented, old thermoino firmware']);
            otherwise
                IOPort('Close',s);
                error(['Version ' ver ' not supported']);
        end
        
        %-----------------------------------------------------------
    case 'time'  % UseThermoino('time',Index) --> returns the time (ms) of thermoino Index
        % UseThermoino('time') --> returns the time (ms) of 1st thermoino
        %-----------------------------------------------------------
        if numel(varargin) == 1
            ind = varargin{1};
        else
            ind = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,"GETTIME");
            varargout{1} = str2num(resp);
            varargout{2} = when;
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'id'  % UseThermoino('id',Index) --> returns the name of thermoino Index
        % UseThermoino('id') --> returns the name of 1st thermoino
        %-----------------------------------------------------------
        if numel(varargin) == 1
            ind = varargin{1};
        else
            ind = 1;
        end
        if ind <= numel(thermoino)
            varargout{1} = mywriteread(thermoino(ind).handle,"GETID");
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        %-----------------------------------------------------------
    case 'trigger' % UseThermoino('trigger',Index) --> START thermoino Index
        % UseThermoino('trigger') --> START 1st thermoino
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        if numel(varargin) == 1
            ind = varargin{1};
        else
            ind = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,"START");
            status = str2num(resp);
            if status > 0
                thermoino(ind).t = thermoino(ind).t_init; %temp back to where we started
                varargout{1}     = when;
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'move' % UseThermoino('move',dur, Index) --> MOVE (dur in 탎) thermoino Index
        % UseThermoino('move',dur) --> MOVE (dur in 탎) 1st thermoino
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        p_dur = varargin{1};
        if numel(varargin) == 2
            ind   = varargin{2};
        else
            ind = 1;
        end
        if ind <= numel(thermoino)
            outCmd = sprintf('MOVE;%d',round(p_dur));
            [resp, when] = mywriteread(thermoino(ind).handle,outCmd);
            status = str2num(resp);
            if status > 0
                update_thermoino(ind,p_dur)
                varargout{1} = when;
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        %-----------------------------------------------------------
    case 'set'  % UseThermoino('set',TargetTemp, Index) --> MOVE thermoino Index to TargetTemp in 캜
        % UseThermoino('set',TargetTemp) --> MOVE 1st thermoino to TargetTemp in 캜
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        target = varargin{1};
        if numel(varargin) == 2 %just a temperature
            ind    = varargin{2};
        else
            ind    = 1;
        end
        if ind <= numel(thermoino)
            p_dur = ((target - thermoino(ind).t)./thermoino(ind).ror).*1e6;
            outCmd = sprintf('MOVE;%d',round(p_dur));
            [resp, when] = mywriteread(thermoino(ind).handle,outCmd);
            status = str2num(resp);
            if status > 0
                update_thermoino(ind,p_dur)
                varargout{1} = when;
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'shock'  % UseThermoino('shock', nShocks, ISI, Index) --> apply nShocks with ISI (thermoino Index)
        % UseThermoino('shock', nShocks, ISI) --> apply nShocks with ISI (1st thermoino)
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        nShocks = varargin{1};
        isi     = varargin{2};
        if numel(varargin) == 3
            ind     = varargin{3};
        else
            ind     = 1;
        end
        
        if isi < min_isi, error(['Minimum ISI is ' num2str(min_isi) '탎']);end
        
        outCmd = sprintf('SHOCK;%d;%d\n',round(nShocks),round(isi));
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,outCmd);
            status = str2num(resp);
            if status > 0
                varargout{1} = when;
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'initctc' %
        %-----------------------------------------------------------
        error(['INITCTC is obsolete use new LOADCTC to also init the CTC'])
        
        %-----------------------------------------------------------
    case 'maxctc'  % UseThermoinoPTB('maxctc',period,Index) --> returns the # of possible CTC entries given period
        % UseThermoinoPTB('maxctc',period) --> returns the # of possible CTC entries
        %-----------------------------------------------------------
        period  = varargin{1};
        if numel(varargin) == 2
            ind = varargin{2};
        else
            ind = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,sprintf('MAXCTC;%d',period));
            status = str2num(resp);
            if status > 0
                varargout{1} = status;                
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'loadctc'% UseThermoino('loadctc', period, data, Index) --> uploads data to thermoino Index and sets period (all in ms)
        % UseThermoino('loadctc', period, data)  --> uploads data to 1st thermoino and sets period (all in ms)
        %-----------------------------------------------------------
        period  = varargin{1};
        data    = varargin{2};
        if numel(varargin) == 3
            ind     = varargin{3};
        else
            ind     = 1;
        end
        
        if period > maxperiod, error([num2str(period) ' exceeds max period of ' num2str(maxperiod)]);end
        if any(abs(data)>period), error(['No pulse can be longer than the CTC period of ' num2str(period)]);end
        if any(~isInt(data)), error(['Only integers allowed']);end
        
        if ind <= numel(thermoino)
            outCmd = sprintf('INITCTC;%d',round(period)); % we always initialize the CTC
            [resp, ~] = mywriteread(thermoino(ind).handle,outCmd);
            if str2num(resp)<0
                error('thermoino busy');
                return;
            end
            
            outCmd = sprintf('MAXCTC;%d',round(period)); % see how many entries fit
            [resp, ~] = mywriteread(thermoino(ind).handle,outCmd);
            maxctc = str2num(resp);
            if numel(data) > maxctc, error(['A maximum of ' num2str(maxctc) ' pulses is allowed']);end
            
            for m = 1:numel(data)
                outCmd = sprintf('LOADCTC;%d',data(m));
                [resp, ~] = mywriteread(thermoino(ind).handle,outCmd);
                if str2num(resp)<0
                    error(ERRORMSG{abs(str2num(resp))});
                    return; % takes about 8.3s for 2500 entries
                end
                %[n, ~, ~, ~, ~, ~] = IOPort('Write', thermoino(ind).handle,sprintf('%s\r\n',outCmd)); %takes about 2.9s for 2500 entries
                
            end
            thermoino(ind).ctc    = data(:);
            thermoino(ind).period = period;
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        %-----------------------------------------------------------
    case 'queryctc' % UseThermoino('queryctc', Index) --> read CTC data from thermoino Index
        % UseThermoino('queryctc') --> read CTC data from 1st thermoino
        % returns CTC data as a n x 2 matrix column 1: 0..n-1, column 2: data
        %-----------------------------------------------------------
        if numel(varargin) == 1
            ind     = varargin{1};
        else
            ind     = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,'QUERYCTC;3',3);
            status = str2num(resp);
            if status > 0
                % now we start reading the data
                data = [];
                a = IOPort('BytesAvailable',thermoino(ind).handle);
                while a>0
                    data = [data IOPort('Read', thermoino(ind).handle)];
                    pause(0.01);
                    a = IOPort('BytesAvailable',thermoino(ind).handle);
                end
                varargout{1} = str2num(char(data));                
            else
                varargout{1} = ERRORMSG{abs(status)};
            end            
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        %-----------------------------------------------------------
    case 'execctc'% UseThermoino('execctc', Index) --> start CTC on thermoino Index
        % UseThermoino('execctc')  --> start CTC on 1st thermoino Index
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        if numel(varargin) == 1
            ind     = varargin{1};
        else
            ind     = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,'EXECCTC');
            status = str2num(resp);
            if status > 0
                varargout{1} = when;
                p_dur = sum(thermoino(ind).ctc)*1e3; %in 탎
                update_thermoino(ind,p_dur)
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        
        %-----------------------------------------------------------
    case 'flushctc'% UseThermoino('flushctc', Index) --> clear CTC data in thermoino Index
        % UseThermoino('flushctc')  --> clear CTC data in 1st thermoino
        % returns PTB time of start (or error string e.g. if busy)
        %-----------------------------------------------------------
        
        if numel(varargin) == 1
            ind     = varargin{1};
        else
            ind     = 1;
        end
        if ind <= numel(thermoino)
            [resp, when] = mywriteread(thermoino(ind).handle,'FLUSHCTC');
            status = str2num(resp);
            if status > 0
                varargout{1} = when;
                thermoino(ind).ctc    = [];
                thermoino(ind).period = [];
            else
                varargout{1} = ERRORMSG{abs(status)};
            end
        else
            error(['Thermoino ' num2str(ind) ' does not exist'])
        end
        %-----------------------------------------------------------        
    case 'kill' % closes all thermoino instances
        IOPort('CloseAll');
        clear global thermoino
    otherwise
        varargout{1} = 'command not implemented';
end
end

%-----------------------------------------------------
function [resp, when] = mywriteread(s,str,n)
% 3 args : wait for n bytes (including CR and LF)
% 2 args : wait for LF
t_out = 2; %timeout 2s
data  = [];
while(IOPort('BytesAvailable',s)>0)
    trash = IOPort('Read',s); % get rid of stuff in buffer
end
[~, when, ~, ~, ~, ~] = IOPort('Write', s, sprintf('%s\r\n',str));
if nargin==3
    elapsed = 0;
    while (IOPort('BytesAvailable',s) < n) && (elapsed < t_out) %get busy until we have enough data or time_out
        elapsed = GetSecs-when;
    end
    data = IOPort('Read', s, 1, n);
    resp = char(data);
else
    while isempty(data) || (data(end) ~= 10) % wait for LF
        data = [data IOPort('Read', s)];
    end
    resp = char(data(1:end-2));
end
end

function update_thermoino(ind,p_dur)
global thermoino
thermoino(ind).t = thermoino(ind).t + (thermoino(ind).ror*p_dur/1e6); %update internal temp
end

function result = isInt(x)
result = isfinite(x) & (x == floor(x));
end
