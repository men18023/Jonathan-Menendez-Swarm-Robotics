function tag = robotat_3pi_read_apriltag(robot)
    if(robot.tcpsock.NumBytesAvailable > 0)
        tag = jsondecode(readline(robot.tcpsock));
    else
        tag.id = -1;
        tag.x = 0;
        tag.y = 0;
        tag.z = 0;
        tag.rotx = 0;
        tag.roty = 0;
        tag.rotz = 0;
        warning('No new tag received.');
    end
end