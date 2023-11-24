% Create a parallel pool (if not already created)
if isempty(gcp('nocreate'))
    parpool;
end

% Initialize a flag to pause the main loop
pauseFlag = false;

% Create a parallel task with a callback for button press checking
buttonPressTask = parfeval(@waitForRightArrowKeyPress, 2, pauseFlag);

% Main loop
while true
    % Your main code here
    
    % Check if the pause flag has been set
    if pauseFlag
        fprintf('Right arrow key pressed. Pausing the loop...\n');
        break;
    end
    
    % Insert a delay to avoid using too much CPU
    pause(0.1);
end

% Function to check for right arrow key press
function waitForRightArrowKeyPress(pauseFlag)
    while true
        if waitforbuttonpress == 1
            key = get(gcf, 'CurrentCharacter');
            if strcmp(key, 'rightarrow')
                % Set the pause flag to true
                pauseFlag.Value = true;
                return; % Right arrow key pressed, exit the function
            end
        end
    end
end
