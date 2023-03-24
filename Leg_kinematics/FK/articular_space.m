% Define the joint ranges
q1_range = [-230 -160];
q2_range = [-25.4 15];
q3_range = [-230 -160];

% Create a figure to visualize the articular space
figure;
hold on;
xlabel('q1 (degrees)');
ylabel('q2 (degrees)');
zlabel('q3 (degrees)');
title('Articular Space');

% Plot the joint ranges
plot3(q1_range, [q2_range(1) q2_range(1)], q3_range, 'b', 'LineWidth', 2);
plot3(q1_range, [q2_range(2) q2_range(2)], q3_range, 'b', 'LineWidth', 2);
plot3(q1_range, q2_range, [q3_range(1) q3_range(1)], 'r', 'LineWidth', 2);
plot3(q1_range, q2_range, [q3_range(2) q3_range(2)], 'r', 'LineWidth', 2);

% Set the axis limits
xlim([q1_range(1) q1_range(2)]);
ylim([q2_range(1) q2_range(2)]);
zlim([q3_range(1) q3_range(2)]);

% Show the grid
grid on;