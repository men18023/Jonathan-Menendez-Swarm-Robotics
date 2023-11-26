function robotat_3pi_flush_apriltag_buffer(robot)
    if(robot.tcpsock.NumBytesAvailable > 0)
        flush(robot.tcpsock);
    else
        warning('Buffer already empty.');
    end
end