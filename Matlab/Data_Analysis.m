%% Sensor system validation
clear all
close all
clc
t_step=0.005;

%% Bag Read
varname = strings;
filename = "../rosbag/Evaluation_1.bag";
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

% interp1
for i = 1 : length(varname)
    Data_i.(varname(i))=interp1(Data.(['t_' char(varname(i))]),Data.(varname(i)),t_range);
end

Data_i.aruco_pose(1164:1584,3) = Data_i.aruco_pose(1164:1584,3)-2.5;
Data_i.aruco_pose(1584:2000,3) = Data_i.aruco_pose(1584:2000,3)-2;
%Data_i.aruco_pose(:,3) = Data_i.aruco_pose(:,3)-11.5;
Data_i.aruco_pose(:,2) = [];
Data_i.Slip_NoSlip(2962+180:end) = 0;

for i = 1 : length(Data_i.Slip_NoSlip)
   if(Data_i.Slip_NoSlip(i) ~= 1)
       Data_i.Slip_NoSlip(i) = 0;
   end
end

%% Figure
figure(1)
subplot(3,1,1)
set(gcf, 'Position', [500 0 800 1000], 'color', 'white')
yyaxis left
plot(Data_i.aruco_pose(1:4100,1))
hold on
ylim([5 25])
yyaxis right
plot(Data_i.aruco_pose(1:4100,2))
hold off
legend('pitch', 'updown')
grid on
ylim([25 60])

subplot(3,1,2)
plot(Data_i.FSS_sum(180:end))
   
subplot(3,1,3)
stem(Data_i.Slip_NoSlip(180:end))
ylim([-0.1 1.2])
