%% Plot figures for paper 
clear; close all; clc;
% Shared params
blue_color = [0 116 186]/255;
orange_color = [223 80 35]/255;
font_size = 14;

% subplot_margin = 0.08; %0.1;
% subplot_spacing = 0.05; %0.06;
subplot_margin = 0.08;
subplot_spacing_vert = 0.09;
subplot_spacing_hor = 0.02;

%% Make error bar plots (sbsos vs. odom+local)
% loop_vec = 1:4;
loop_vec = [1, 3, 5];
N_vec = [50, 100];
file_name_head = '../data/results/results_odom/results_sbsos_loop_';
file_name_center = '_N_';
file_name_end = '_double_cover_1_noise_max_3.1416_runs_per_noise_10.mat';
% file_name_end = '_double_cover_1_noise_max_0.34907_runs_per_noise_10.mat';
fig = figure();
set(fig,'defaulttextinterpreter','latex');

for jdx_top=1:length(N_vec)
    N = N_vec(jdx_top);
    for idx_top=1:length(loop_vec)
        subaxis(length(N_vec),length(loop_vec), idx_top + (jdx_top-1)*length(loop_vec), ...
            'Margin', subplot_margin, 'SpacingHoriz', subplot_spacing_hor, ...
            'SpacingVert', subplot_spacing_vert);
        loop = loop_vec(idx_top);
        file_name = [file_name_head num2str(loop) file_name_center num2str(N) ...
                     file_name_end];
        load(file_name);
        bar_data = [mean(error_norm_quat_sbsos(2:5, :), 2) mean(error_norm_quat_odom(2:5, :), 2)];
        x_axis_vals = ang_noise_max_vec(2:5)/pi;
        b = bar(x_axis_vals, bar_data, 'grouped');
        hold on;
        set(gca,'TickLabelInterpreter','latex');
        if jdx_top == length(N_vec)
            xlabel('Max. Noise Error ($\pi$ rad)','FontSize', font_size, 'Interpreter', 'latex');
        end
        if idx_top == 1
            ylabel('Mean Quat. Norm Err.','FontSize', font_size, 'Interpreter', 'latex');
        end
        grid minor;
        ax1 = gca;
        b(1).FaceColor = blue_color;
        b(1).FaceAlpha = 0.8;
        b(2).FaceColor = orange_color;
        b(2).FaceAlpha = 0.8;
        ylim([0.15 1.15]);
        if loop==1
            title(['$N = ' num2str(N) ',$ ' num2str(loop) ' Loop Closure'], ... 
                   'FontSize', font_size, 'Interpreter', 'latex');
        else
            title(['$N = ' num2str(N) ',$ ' num2str(loop) ' Loop Closures'], ... 
                   'FontSize', font_size, 'Interpreter', 'latex');
        end
        e_sbsos = errorbar(x_axis_vals-0.016, mean(error_norm_quat_sbsos(2:5, :), 2), ...
        mean(error_norm_quat_sbsos(2:5, :), 2) - quantile(error_norm_quat_sbsos(2:5, :), 0.25, 2),...
        quantile(error_norm_quat_sbsos(2:5, :), 0.75, 2) - mean(error_norm_quat_sbsos(2:5, :), 2),...
        '.', 'LineWidth', 1.5);
        e_sbsos.Marker = 'none';
        e_sbsos.Color = [0 0 0]; %blue_color;
        e_odom = errorbar(x_axis_vals+0.016,mean(error_norm_quat_odom(2:5, :), 2), ...
        mean(error_norm_quat_odom(2:5, :), 2) - quantile(error_norm_quat_odom(2:5, :), 0.25, 2),...
        quantile(error_norm_quat_odom(2:5, :), 0.75, 2) - mean(error_norm_quat_odom(2:5, :), 2),...
        '.', 'LineWidth', 1.5,'MarkerSize', 0.000001);
        e_odom.Marker = 'none';
        e_odom.Color = [0 0 0]; %orange_color;
        if idx_top==1 && jdx_top == 1
            lgnd = legend([b(1) b(2)], 'SBSOS','Local', 'Location', 'NorthWest');
            set(lgnd, 'Interpreter', 'Latex','FontSize', font_size);
            set(lgnd.BoxFace, 'ColorType', 'truecoloralpha', 'ColorData', uint8([255;255;255;0.8*255]));
        end
    end
end

%% Print bar graphs to PDF
% set(fig,'PaperSize',[20 10]);
% print(fig,'../fig/error_results_50_100_135','-dpdf','-r0');

%% Make cost function plots (sbsos vs. odom+local)
% loop_vec = 1:4;
loop_vec = [1, 3, 5];
N_vec = [50, 100];
file_name_head = '../data/results/results_odom/results_sbsos_loop_';
file_name_center = '_N_';
file_name_end = '_double_cover_1_noise_max_3.1416_runs_per_noise_10.mat';
% file_name_end = '_double_cover_1_noise_max_0.34907_runs_per_noise_10.mat';
fig = figure();
set(fig,'defaulttextinterpreter','latex');

for jdx_top=1:length(N_vec)
    N = N_vec(jdx_top);
    for idx_top=1:length(loop_vec)
        subaxis(length(N_vec),length(loop_vec), idx_top + (jdx_top-1)*length(loop_vec), ...
            'Margin', subplot_margin, 'SpacingHoriz', subplot_spacing_hor, ...
            'SpacingVert', subplot_spacing_vert);
        loop = loop_vec(idx_top);
        file_name = [file_name_head num2str(loop) file_name_center num2str(N) ...
                     file_name_end];
        load(file_name);
        bar_data = [mean(f_results_sbsos(2:5, :), 2) mean(f_results_odom(2:5, :), 2)];
        x_axis_vals = ang_noise_max_vec(2:5)/pi;
        b = bar(x_axis_vals, bar_data, 'grouped');
        hold on;
        set(gca,'TickLabelInterpreter','latex');
        if jdx_top == length(N_vec)
            xlabel('Max. Noise Error ($\pi$ rad)','FontSize', font_size, 'Interpreter', 'latex');
        end
        if idx_top == 1
            ylabel('Objective Function','FontSize', font_size, 'Interpreter', 'latex');
        end
        grid minor;
        ax1 = gca;
        b(1).FaceColor = blue_color;
        b(1).FaceAlpha = 0.8;
        b(2).FaceColor = orange_color;
        b(2).FaceAlpha = 0.8;
        if N == 50
            ylim([0 1.25]);
        else
            ylim([0 1.6]);
        end
        if loop==1
            title(['$N = ' num2str(N) ',$ ' num2str(loop) ' Loop Closure'], ... 
                   'FontSize', font_size, 'Interpreter', 'latex');
        else
            title(['$N = ' num2str(N) ',$ ' num2str(loop) ' Loop Closures'], ... 
                   'FontSize', font_size, 'Interpreter', 'latex');
        end
        e_sbsos = errorbar(x_axis_vals-0.016, mean(f_results_sbsos(2:5, :), 2), ...
        mean(f_results_sbsos(2:5, :), 2) - quantile(f_results_sbsos(2:5, :), 0.25, 2),...
        quantile(f_results_sbsos(2:5, :), 0.75, 2) - mean(f_results_sbsos(2:5, :), 2),...
        '.', 'LineWidth', 1.5);
        e_sbsos.Marker = 'none';
        e_sbsos.Color = [0 0 0]; %blue_color;
        e_odom = errorbar(x_axis_vals+0.016,mean(f_results_odom(2:5, :), 2), ...
        mean(f_results_odom(2:5, :), 2) - quantile(f_results_odom(2:5, :), 0.25, 2),...
        quantile(f_results_odom(2:5, :), 0.75, 2) - mean(f_results_odom(2:5, :), 2),...
        '.', 'LineWidth', 1.5,'MarkerSize', 0.000001);
        e_odom.Marker = 'none';
        e_odom.Color = [0 0 0]; %orange_color;
        if idx_top==1 && jdx_top == 1
            lgnd = legend([b(1) b(2)], 'SBSOS','Local', 'Location', 'NorthWest');
            set(lgnd, 'Interpreter', 'Latex','FontSize', font_size);
            set(lgnd.BoxFace, 'ColorType', 'truecoloralpha', 'ColorData', uint8([255;255;255;0.8*255]));
        end
    end
end

%% Print bar graphs to PDF
% set(fig,'PaperSize',[20 10]);
% print(fig,'../fig/cost_function_results_50_100_135','-dpdf','-r0');
