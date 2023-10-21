% Define the number of workers
N = 10;
combinedVector = zeros(1, N);
combinedxVector = zeros(1, N);
num = zeros(1,N);
x = zeros(1,N);
% Define the number of iterations
numIterations = 10; % Set to the desired number of iterations

% Create a matrix to store the history of combined vectors
history = zeros(numIterations, N);
historyx = zeros(numIterations, N);
% Initialize a parallel pool with the specified number of workers
% pool = parpool(N);
pause(1)
for iteration = 1:numIterations
    spmd
        % Generate a single random number for each worker
        [randomNumber,count] = printRandoms(1,10,1);
        num(labindex) = randomNumber;
        x(labindex) = count;
        combinedResult = gcat(num(labindex), 1, 1);
        combinedx = gcat(x(labindex), 2, 1);
    end
    
    % Combine the results from all workers into a single vector
    combinedResult1 = gcat(num(1:N), 1, 1);
    combinedx1 = gcat(x(1:N), 2, 1);
    % Extract the 1xn part from each cell and store it in combinedVector
    for n = 1:N
        combinedVector(n) = combinedResult1{n}(n);
        combinedxVector(n) = combinedx1{n}(n);
    end
    
    % Store the combinedVector in the history matrix
    history(iteration, :) = combinedVector;
    historyx(iteration,:) = combinedxVector;
end

% Store the final combined vector in the last row of the history matrix
history(numIterations, :) = combinedVector;
historyx(numIterations, :) = combinedxVector;
% if iteration
%     robotat_3pi_force_stop(robot7);
%     robotat_3pi_disconnect(robot7);
