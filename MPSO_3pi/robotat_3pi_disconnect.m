function robotat_3pi_disconnect(robot)
    robotat_3pi_force_stop(robot);
    evalin('base', ['clear ', inputname(1)]);
    disp('Disconnected from robot.');
end