goal_1 = robotat_get_pose(robotat,1,ang_seq);
goal = [1,-1];
for gen = first_agent:last_agent
    numb = gen-first_agent+1;
    %eval(['pos_origin' num2str(gen) '=robotat_get_pose(robotat,con,ang_seq);']);
    pos_origin = robotat_get_pose(robotat,gen,ang_seq);
    pos_origin_temp = pos_origin(1:2);
    eval(['tray_x' num2str(gen) '=linspace(pos_origin_temp(1), goal(1), 200);']);
    %tray_x7 = linspace(pos_origin(1), goal(1), 200);
    eval(['tray_y' num2str(gen) '=linspace(pos_origin_temp(2), goal(2), 200);']);
    x_temp = eval(['tray_x' num2str(gen)]);
    y_temp = eval(['tray_y' num2str(gen)]);
    %tray_y7 = linspace(pos_origin(2), goal(2), 200);
    eval(['tray{' num2str(numb) '}' '=[transpose(x_temp),transpose(y_temp)];']);
end