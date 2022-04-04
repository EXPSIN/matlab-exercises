%{
    
%}
function t = graphic_client(t, fsim)

if(isempty(t))
%     t = tcpclient('localhost', 4000);
    try
        t = tcpip('127.0.0.1', 4001, 'timeout', 0.5);
        flushinput(t);
        flushoutput(t);
        fopen(t);
    catch
        t = [];
        warning('You should run the graphic server or check the address and port.');
        return;
    end
    
    
%     % struction -> json
%     cmd_state = 'init';
% 
%     % transfer the json code to server.
%     fprintf(t, jsonencode(cmd_state));
end

% struction -> json
text = jsonencode(fsim);

% transfer the json code to server.
fprintf(t, text);

% data = fscanf(t)

end