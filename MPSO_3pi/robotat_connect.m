function tcp_obj = robotat_connect()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    ip = '192.168.50.200';
    port = 1883;
    try
        tcp_obj = tcpclient(ip, port, 'Timeout', 10);
        % Verify the connection
        if isempty(tcp_obj) || ~isvalid(tcp_obj) || ~isopen(tcp_obj)
            error('TCP connection is not valid or open.');
        end
    catch
        disp('ERROR: Could not connect to Robotat server.');
    end
end