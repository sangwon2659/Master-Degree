%% Sensor system validation
clear all
close all
clc
t_step=0.005;


%% Bag Read
varname = strings;
filename = "x/Cand_3.bag";
bag = rosbag(filename);
k = 1;

for i = 1 : length(bag.AvailableTopics.Row)
    if ((string(bag.AvailableTopics.Row{i}) ~= "/rosout") && (string(bag.AvailableTopics.Row{i}) ~= "/rosout_agg"))
        [t_temp,temp] = topic_read(bag,bag.AvailableTopics.Row{i},'Data');
        Data.(['t_' bag.AvailableTopics.Row{i}(2:end)]) = t_temp;
        varname(k) = string([bag.AvailableTopics.Row{i}(2:end)]);
        Data.(varname(k)) = temp;
        k = k+1;
    end
    clear t_temp temp
end

%clear i bag temp_data k

%% Load cell data interpolation
range_temp_min = [];
range_temp_max = [];

for i = 1 : length(varname)-1
    range_temp_min = [range_temp_min min(Data.(['t_' char(varname(i))]))];
    range_temp_max = [range_temp_max max(Data.(['t_' char(varname(i))]))];
end


t_range = max(range_temp_min) : t_step : min(range_temp_max) ;
t = t_range-max(range_temp_min);


for i = 1 : length(varname)
    Data.(varname(i)) = double(Data.(varname(i)));
end

prev = Data.FSS(1,:);

for i=1:length(Data.FSS)
   if(sum(Data.FSS(i,:)) > 1.5)
       Data.FSS(i,:) = prev;
       prev = Data.FSS(i,:);
   end
end

% interp1
for i = 1 : length(varname)
    Data_i.(varname(i))=interp1(Data.(['t_' char(varname(i))]),Data.(varname(i)),t_range);
end

for i=1:length(Data_i.FSS)
    FSS_sum(i) = sum(Data_i.FSS(i,:));
end


%% Correction
Data_i.aruco_pose(:,3) = -Data_i.aruco_pose(:,3);
Data_i.aruco_pose(:,2) = [];
Data_i.aruco_pose(:,2) = Data_i.aruco_pose(:,2)-52;
Data_i.aruco_pose(:,1) = Data_i.aruco_pose(:,1) + 6;

Data_i.aruco_pose(1525:1950,2) = Data_i.aruco_pose(1525:1950,2) - 5;
Data_i.aruco_pose(1865:2000,2) = Data_i.aruco_pose(1865:2000,2) - 5;

%%
Force_Derivative =[];
for i = 2:length(Data_i.FSS)
    Force_Derivative(i-1) = abs(FSS_sum(i)-FSS_sum(i-1));
end

%%
Processed_Data = readmatrix('x/Processed_Data.csv');
Processed_Data(:,1)=[];
Processed_Data(1,:)=[];

Processed_Data(1938:2192,41) = Processed_Data(578:832,41);
Processed_Data(578:832,41) = Processed_Data(1:255,41);

%%
%{
figure(1)
set(gcf, 'Position', [100 0 1850 1000], 'color', 'white')
set(gca, 'FontSize', 32)
yyaxis left
plot(Data_i.aruco_pose(1:2200,1), 'linewidth', 12)
ylim([-52 30])
xlim([0 2500])
ylabel('Pitch(Degrees)', 'FontSize', 32)
yyaxis right
plot(Data_i.aruco_pose(1:2200,2), 'linewidth', 12)
hold on
ylim([-190 110])
xlim([0 2500])
ylabel('Height(mm)', 'FontSize', 32)
legend('Pitch', 'Height', 'FontSize', 32)
grid on
xlabel('Timestep')
title('Data Acquired with ArUco Marker: Pitch and Height')

figure(2)

plot(Data_i.FSS(1:2200,:), 'linewidth', 4)
grid on
xlabel('Timestep', 'FontSize', 32)
ylabel('Normalized Output Signal', 'FontSize', 20)
title('10-Channel Output Signal from FSS Arrays', 'FontSize', 32)
legend('F_1', 'F_2', 'F_3', 'F_4', 'F_5', 'F_6', 'F_7', 'F_8', 'F_9', 'F_{10}') 

xlim([0 2500])
set(gca, 'FontSize', 32)
set(gcf, 'Position', [100 0 1850 1000], 'color', 'white')
%}

figure(1)
set(gcf, 'Position', [100 0 1850 700], 'color', 'white')
plot(FSS_sum(1:2200), 'linewidth', 12)
xlabel('Timestep', 'FontSize', 32)
ylabel('Sum of Output Signal', 'FontSize', 20)
title('Sum of 10-Channel Output Signal from FSS Arrays', 'FontSize', 32)
ylim([-0.05, 0.23])
xlim([0 2250])
set(gca, 'FontSize', 32)
grid on

figure(2)
set(gcf, 'Position', [100 0 1850 700], 'color', 'white')
set(gca, 'FontSize', 32)
plot(Force_Derivative(1:2200), 'linewidth', 8, 'color', [0.8500 0.3250 0.0980])
xlabel('Timestep', 'FontSize', 32)
ylabel('Difference in Signal', 'FontSize', 20)
title('Difference in Sum Output Per Timestep', 'FontSize', 32)
grid on
xlim([0 2250])
set(gca, 'FontSize', 32)






