%{

fig_number, The fi
%}
function H_pos = graphic_position(H_pos, fsim, fig_number)
if(isempty(H_pos))
    figure(fig_number); clf;
    H_pos.fig = gcf;
    H_pos.traj = animatedline('color', 'k');
    H_pos.traj2 = animatedline('color', 'r');
end
addpoints(H_pos.traj, fsim.t, fsim.x);
addpoints(H_pos.traj2, fsim.t, fsim.y);

end