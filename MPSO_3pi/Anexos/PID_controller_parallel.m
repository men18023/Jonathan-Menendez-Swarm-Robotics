function phi = PID_controller_parallel(obj_tcp, Agent_list, offset_list, traj_list)
    % Initialize the parallel pool
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool;
    end

    % Properties of the robot
    MAX_WHEEL_VELOCITY = 800;
    WHEEL_RADIUS = 32 / 2000;
    MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;
    DISTANCE_FROM_CENTER = 96 / 2000;
    limiter = 50;

    % Parameters for PID controllers
    v0 = MAX_SPEED / 4;
    alpha = 0.9;
    kpO = 1;
    kiO = 0.001;
    kdO = 0;

    % Initialize variables
    phi = zeros(length(Agent_list), 2);

    % Create a parallel pool if none exists
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        poolobj = parpool;
    end

    % Create a cell array to store the futures
    futures = cell(length(Agent_list), 1);

    % Use parfeval to asynchronously execute the function in parallel
    for i = 1:length(Agent_list)
        agent = Agent_list(i);
        offset = offset_list(i);
        traj = traj_list{i};

        % Asynchronously execute the function using parfeval
        futures{i} = parfeval(@PID_controller_worker, 3, obj_tcp, agent, offset, traj);

        % Display a message for demonstration purposes
        fprintf('Task for Agent %d submitted.\n', agent);
    end

    % Retrieve results from the futures
    for i = 1:length(Agent_list)
        % Fetch the results from the completed futures
        [completedIdx, phi_result] = fetchNext(futures);

        % Store the results in the output variable
        phi(completedIdx, :) = phi_result;
    end
end

% Worker function that performs the calculations for one agent
function phi_result = PID_controller_worker(obj_tcp, agent, offset, traj)
    % Perform the calculations for one agent
    % Modify this function based on your original PID_controller1 function
    % ...

    % Example: Call your original PID_controller1 function
    [phi_result, ~, ~] = PID_controller1(obj_tcp, agent, offset, traj, 0);

    % Display a message for demonstration purposes
    fprintf('Task for Agent %d completed.\n', agent);
end


