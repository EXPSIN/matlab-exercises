%{
    The script is used to generate some figures to show the simulation process.
    This server received json data with tcpip
%}
clear all; close all; clc; 


%{
address:    0.0.0.0 接收任何地址
port:       4001
Timeout:    1 second
%}
t = tcpip('0.0.0.0', 4001, 'NetworkRole','Server', 'Timeout', 0.5); % 
flushinput(t);
flushoutput(t);

% Start the server.
fopen(t);

% Create the handle of graphic
graphic_handle = [];
while(true)
    
    % Read data from client
    json_data = fscanf(t);
    
    try
        % Decode the data to struct.
        fsim = jsondecode(json_data);
        
        % Draw data
        graphic_handle    = call_graphic(graphic_handle, fsim);
    catch
        warning('Client may exit.');
        
        % restart the tcpip server
        fclose(t); fopen(t);
        
        % restart the figures.
        graphic_handle = [];
    end
    
    
end

%{
    
%}
function H = call_graphic(H, fsim)
% 1. The initial process.
if(isempty(H))
    H.pos   = []; 
    H.decision_model = []; 
end

% 2. The update process 
H.pos = graphic_position(H.pos, fsim, 1);   % The position graphic use figure(1);


% 3. Draw the upadated data.
drawnow limitrate;
end

