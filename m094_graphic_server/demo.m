clear all; close all; clc; 


fsim.t = 0;
fsim.x = 0;
fsim.y = 0;


t = graphic_client([], fsim);


for x = 0:0.1:100000
    fsim.t = x;
    fsim.x = cos(x);
    fsim.y = sin(x);
    fprintf('%f\n', x);
    
    
    t = graphic_client(t, fsim);
    pause(0.1);
end


