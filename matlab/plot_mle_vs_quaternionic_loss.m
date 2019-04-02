%% plot_mle_vs_quaternionic_loss
clear; close all; clc


%% Params
N = 1000;
thetas = linspace(-pi, pi, N);
mle_loss = zeros(N, 1);
q_loss =zeros(N, 1);
axis = rand(3,1);
axis = axis/norm(axis,2);
R_base = eye(3);
q_base = bot_matrix_to_quat(R_base);
%% Compute losses
for idx=1:length(thetas)
    
    theta = thetas(idx);
    R = axis_angle(axis, theta);
    mle_loss(idx) = norm(R_base-R, 'fro')^2;
    q = bot_matrix_to_quat(R);
    q_loss(idx) = norm(q_base-q,2)^2;
    
end

%% Plot
font_size = 16;
blue_color = [0 116 186]/255;
orange_color = [223 80 35]/255;
fig = figure;
set(fig,'defaulttextinterpreter','latex');
plot(thetas/pi, mle_loss, 'LineWidth', 2.5, 'Color', blue_color);
hold on;
plot(thetas/pi, q_loss, 'LineWidth', 2.5, 'Color', orange_color);
grid on;
set(gca,'TickLabelInterpreter','latex');
%title('Langevins MLE vs. Quaternionic Loss');
xlabel('rotation angle ($\pi$ rad)', 'FontSize', font_size+2, 'Interpreter', 'latex');
ylabel('loss', 'FontSize', font_size+2, 'Interpreter', 'latex');
lgnd = legend({'MLE Loss', 'Quaternionic Loss'}, 'Location', 'NorthEast');
set(lgnd, 'Interpreter', 'Latex','FontSize', font_size);


