% Parameters
maxSpeed = 300; % Max speed of the train in km/h
acceleration = 1.5; % Acceleration in m/s^2
deceleration = -2.0; % Deceleration in m/s^2
timeStep = 0.1; % Time step for the simulation (seconds)
totalTime = 600; % Total simulation time in seconds

% Time vector
t = 0:timeStep:totalTime;
speed = zeros(size(t)); % Speed array initialization

% Loop through each time step to control speed dynamically
for i = 2:length(t)
    if t(i) < 200 % Accelerating phase
        speed(i) = speed(i-1) + acceleration * timeStep;
    elseif t(i) >= 200 && t(i) < 400 % Constant speed phase
        speed(i) = maxSpeed;
    else % Decelerating phase
        speed(i) = speed(i-1) + deceleration * timeStep;
        if speed(i) < 0 % Ensure speed doesn’t go below zero
            speed(i) = 0;
        end
    end
end

% Plot the speed curve
plot(t, speed, 'LineWidth', 2);
title('High-Speed Train Control: Speed vs Time');
xlabel('Time (seconds)');
ylabel('Speed (km/h)');
grid on;
% PID control for speed regulation
desiredSpeed = 250; % Set desired speed in km/h
Kp = 0.5; Ki = 0.01; Kd = 0.1; % PID gains

% Implement PID controller
pidController = pid(Kp, Ki, Kd);
speedControl = feedback(pidController, 1);

% Simulate the system
t_sim = 0:0.1:100; % Time vector
referenceSpeed = desiredSpeed * ones(size(t_sim)); % Reference signal
[y, t_sim] = lsim(speedControl, referenceSpeed, t_sim);

plot(t_sim, y, 'LineWidth', 2);
title('PID Controlled Speed');
xlabel('Time (s)');
ylabel('Train Speed (km/h)');
grid on;
curveRadius = 500; % Radius of a curve in meters
safetyFactor = 0.75; % To reduce speed on curves for safety
curveSpeed = sqrt(curveRadius * 9.81 * safetyFactor); % Calculate safe speed

% Adjust speed during the curve
for i = 1:length(t)
    if isCurve(t(i)) % isCurve is a function to check if the train is on a curve
        speed(i) = min(speed(i), curveSpeed);
    end
end
% Distance to another train
distanceToTrain = detectTrainPosition(); % Function to detect nearby trains

% If train is too close, start braking
if distanceToTrain < safeDistance
    speed(i) = speed(i-1) + emergencyBrakeRate * timeStep;
end
