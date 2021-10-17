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

%% ArUco
%Data Correction
Data_i.aruco_pose(:,3) = -Data_i.aruco_pose(:,3);
Data_i.aruco_pose(:,2) = [];
Data_i.aruco_pose(:,2) = Data_i.aruco_pose(:,2)-52;
Data_i.aruco_pose(:,1) = Data_i.aruco_pose(:,1) + 6;

Data_i.aruco_pose(1525:1950,2) = Data_i.aruco_pose(1525:1950,2) - 5;
Data_i.aruco_pose(1865:2000,2) = Data_i.aruco_pose(1865:2000,2) - 5;

%% Force Sum
for i=1:length(Data_i.FSS)
    FSS_sum(i) = sum(Data_i.FSS(i,:));
end

%% Force Derivative
Force_Derivative =[];
for i = 2:length(Data_i.FSS)
    Force_Derivative(i-1) = abs(FSS_sum(i)-FSS_sum(i-1));
end

%% Covariance (n=10)
FSS_Together = Data_i.FSS;

for i=11:length(Data_i.FSS)
    FFT_diff = [FSS_Together(i,:)-FSS_Together(i-1,:);...
        FSS_Together(i-1,:)-FSS_Together(i-2,:);...
        FSS_Together(i-2,:)-FSS_Together(i-3,:);...
        FSS_Together(i-3,:)-FSS_Together(i-4,:);...
        FSS_Together(i-4,:)-FSS_Together(i-5,:);...
        FSS_Together(i-5,:)-FSS_Together(i-6,:);...
        FSS_Together(i-6,:)-FSS_Together(i-7,:);...
        FSS_Together(i-7,:)-FSS_Together(i-8,:);...
        FSS_Together(i-8,:)-FSS_Together(i-9,:);...
        FSS_Together(i-9,:)-FSS_Together(i-10,:)];

    FFT_diff = FFT_diff;
    temp = transpose(FFT_diff)*FFT_diff;
    Cov(i) = sum(temp(:)) - trace(temp); 
end

%Covariance Correction
for i=1500:2000
    Cov(i) = Cov(i)*15;
end

%% Covariance for 3 Window Sizes
FSS_sum = sum(transpose(FSS_Together));

Cov_Array_1 = [];
Cov_Array_2 = [];
Cov_Array_3 = [];

for i = 11:length(Data_i.FSS)
    
    FFT_diff = [FSS_Together(i,1)-FSS_Together(i-1,1)...
    FSS_Together(i,2)-FSS_Together(i-1,2)...
    FSS_Together(i,3)-FSS_Together(i-1,3)...
    FSS_Together(i,4)-FSS_Together(i-1,4)...
    FSS_Together(i,5)-FSS_Together(i-1,5)...
    FSS_Together(i,6)-FSS_Together(i-1,6)...
    FSS_Together(i,7)-FSS_Together(i-1,7)...
    FSS_Together(i,8)-FSS_Together(i-1,8)...
    FSS_Together(i,9)-FSS_Together(i-1,9)...
    FSS_Together(i,10)-FSS_Together(i-1,10)];
         
    temp = transpose(FFT_diff)*FFT_diff;
    Cov = sum(temp(:)) - trace(temp);
    Cov_Array_1(i) = Cov;
    
    FFT_diff_2 = [FSS_Together(i,1)-FSS_Together(i-1,1)...
        FSS_Together(i,2)-FSS_Together(i-1,2)...
        FSS_Together(i,3)-FSS_Together(i-1,3)...
        FSS_Together(i,4)-FSS_Together(i-1,4)...
        FSS_Together(i,5)-FSS_Together(i-1,5)...
        FSS_Together(i,6)-FSS_Together(i-1,6)...
        FSS_Together(i,7)-FSS_Together(i-1,7)...
        FSS_Together(i,8)-FSS_Together(i-1,8)...
        FSS_Together(i,9)-FSS_Together(i-1,9)...
        FSS_Together(i,10)-FSS_Together(i-1,10);...
        
        FSS_Together(i-1,1)-FSS_Together(i-2,1)...
        FSS_Together(i-1,2)-FSS_Together(i-2,2)...
        FSS_Together(i-1,3)-FSS_Together(i-2,3)...
        FSS_Together(i-1,4)-FSS_Together(i-2,4)...
        FSS_Together(i-1,5)-FSS_Together(i-2,5)...
        FSS_Together(i-1,6)-FSS_Together(i-2,6)...
        FSS_Together(i-1,7)-FSS_Together(i-2,7)...
        FSS_Together(i-1,8)-FSS_Together(i-2,8)...
        FSS_Together(i-1,9)-FSS_Together(i-2,9)...
        FSS_Together(i-1,10)-FSS_Together(i-2,10);...
        
        FSS_Together(i-2,1)-FSS_Together(i-3,1)...
        FSS_Together(i-2,2)-FSS_Together(i-3,2)...
        FSS_Together(i-2,3)-FSS_Together(i-3,3)...
        FSS_Together(i-2,4)-FSS_Together(i-3,4)...
        FSS_Together(i-2,5)-FSS_Together(i-3,5)...
        FSS_Together(i-2,6)-FSS_Together(i-3,6)...
        FSS_Together(i-2,7)-FSS_Together(i-3,7)...
        FSS_Together(i-2,8)-FSS_Together(i-3,8)...
        FSS_Together(i-2,9)-FSS_Together(i-3,9)...
        FSS_Together(i-2,10)-FSS_Together(i-3,10);...
        
        FSS_Together(i-3,1)-FSS_Together(i-4,1)...
        FSS_Together(i-3,2)-FSS_Together(i-4,2)...
        FSS_Together(i-3,3)-FSS_Together(i-4,3)...
        FSS_Together(i-3,4)-FSS_Together(i-4,4)...
        FSS_Together(i-3,5)-FSS_Together(i-4,5)...
        FSS_Together(i-3,6)-FSS_Together(i-4,6)...
        FSS_Together(i-3,7)-FSS_Together(i-4,7)...
        FSS_Together(i-3,8)-FSS_Together(i-4,8)...
        FSS_Together(i-3,9)-FSS_Together(i-4,9)...
        FSS_Together(i-3,10)-FSS_Together(i-4,10);...
        
        FSS_Together(i-4,1)-FSS_Together(i-5,1)...
        FSS_Together(i-4,2)-FSS_Together(i-5,2)...
        FSS_Together(i-4,3)-FSS_Together(i-5,3)...
        FSS_Together(i-4,4)-FSS_Together(i-5,4)...
        FSS_Together(i-4,5)-FSS_Together(i-5,5)...
        FSS_Together(i-4,6)-FSS_Together(i-5,6)...
        FSS_Together(i-4,7)-FSS_Together(i-5,7)...
        FSS_Together(i-4,8)-FSS_Together(i-5,8)...
        FSS_Together(i-4,9)-FSS_Together(i-5,9)...
        FSS_Together(i-4,10)-FSS_Together(i-5,10)];
    
    temp_2 = transpose(FFT_diff_2)*FFT_diff_2;
    Cov_2 = sum(temp_2(:)) - trace(temp_2);
    Cov_Array_2(i) = Cov_2;
    
    FFT_diff_3 = [FSS_Together(i,1)-FSS_Together(i-1,1)...
        FSS_Together(i,2)-FSS_Together(i-1,2)...
        FSS_Together(i,3)-FSS_Together(i-1,3)...
        FSS_Together(i,4)-FSS_Together(i-1,4)...
        FSS_Together(i,5)-FSS_Together(i-1,5)...
        FSS_Together(i,6)-FSS_Together(i-1,6)...
        FSS_Together(i,7)-FSS_Together(i-1,7)...
        FSS_Together(i,8)-FSS_Together(i-1,8)...
        FSS_Together(i,9)-FSS_Together(i-1,9)...
        FSS_Together(i,10)-FSS_Together(i-1,10);...
        
        FSS_Together(i-1,1)-FSS_Together(i-2,1)...
        FSS_Together(i-1,2)-FSS_Together(i-2,2)...
        FSS_Together(i-1,3)-FSS_Together(i-2,3)...
        FSS_Together(i-1,4)-FSS_Together(i-2,4)...
        FSS_Together(i-1,5)-FSS_Together(i-2,5)...
        FSS_Together(i-1,6)-FSS_Together(i-2,6)...
        FSS_Together(i-1,7)-FSS_Together(i-2,7)...
        FSS_Together(i-1,8)-FSS_Together(i-2,8)...
        FSS_Together(i-1,9)-FSS_Together(i-2,9)...
        FSS_Together(i-1,10)-FSS_Together(i-2,10);...
       
        FSS_Together(i-2,1)-FSS_Together(i-3,1)...
        FSS_Together(i-2,2)-FSS_Together(i-3,2)...
        FSS_Together(i-2,3)-FSS_Together(i-3,3)...
        FSS_Together(i-2,4)-FSS_Together(i-3,4)...
        FSS_Together(i-2,5)-FSS_Together(i-3,5)...
        FSS_Together(i-2,6)-FSS_Together(i-3,6)...
        FSS_Together(i-2,7)-FSS_Together(i-3,7)...
        FSS_Together(i-2,8)-FSS_Together(i-3,8)...
        FSS_Together(i-2,9)-FSS_Together(i-3,9)...
        FSS_Together(i-2,10)-FSS_Together(i-3,10);...
        
        FSS_Together(i-3,1)-FSS_Together(i-4,1)...
        FSS_Together(i-3,2)-FSS_Together(i-4,2)...
        FSS_Together(i-3,3)-FSS_Together(i-4,3)...
        FSS_Together(i-3,4)-FSS_Together(i-4,4)...
        FSS_Together(i-3,5)-FSS_Together(i-4,5)...
        FSS_Together(i-3,6)-FSS_Together(i-4,6)...
        FSS_Together(i-3,7)-FSS_Together(i-4,7)...
        FSS_Together(i-3,8)-FSS_Together(i-4,8)...
        FSS_Together(i-3,9)-FSS_Together(i-4,9)...
        FSS_Together(i-3,10)-FSS_Together(i-4,10);...
        
        FSS_Together(i-4,1)-FSS_Together(i-5,1)...
        FSS_Together(i-4,2)-FSS_Together(i-5,2)...
        FSS_Together(i-4,3)-FSS_Together(i-5,3)...
        FSS_Together(i-4,4)-FSS_Together(i-5,4)...
        FSS_Together(i-4,5)-FSS_Together(i-5,5)...
        FSS_Together(i-4,6)-FSS_Together(i-5,6)...
        FSS_Together(i-4,7)-FSS_Together(i-5,7)...
        FSS_Together(i-4,8)-FSS_Together(i-5,8)...
        FSS_Together(i-4,9)-FSS_Together(i-5,9)...
        FSS_Together(i-4,10)-FSS_Together(i-5,10);...
    
        FSS_Together(i-5,1)-FSS_Together(i-6,1)...
        FSS_Together(i-5,2)-FSS_Together(i-6,2)...
        FSS_Together(i-5,3)-FSS_Together(i-6,3)...
        FSS_Together(i-5,4)-FSS_Together(i-6,4)...
        FSS_Together(i-5,5)-FSS_Together(i-6,5)...
        FSS_Together(i-5,6)-FSS_Together(i-6,6)...
        FSS_Together(i-5,7)-FSS_Together(i-6,7)...
        FSS_Together(i-5,8)-FSS_Together(i-6,8)...
        FSS_Together(i-5,9)-FSS_Together(i-6,9)...
        FSS_Together(i-5,10)-FSS_Together(i-6,10);...
        
        FSS_Together(i-6,1)-FSS_Together(i-7,1)...
        FSS_Together(i-6,2)-FSS_Together(i-7,2)...
        FSS_Together(i-6,3)-FSS_Together(i-7,3)...
        FSS_Together(i-6,4)-FSS_Together(i-7,4)...
        FSS_Together(i-6,5)-FSS_Together(i-7,5)...
        FSS_Together(i-6,6)-FSS_Together(i-7,6)...
        FSS_Together(i-6,7)-FSS_Together(i-7,7)...
        FSS_Together(i-6,8)-FSS_Together(i-7,8)...
        FSS_Together(i-6,9)-FSS_Together(i-7,9)...
        FSS_Together(i-6,10)-FSS_Together(i-7,10);...
        
        FSS_Together(i-7,1)-FSS_Together(i-8,1)...
        FSS_Together(i-7,2)-FSS_Together(i-8,2)...
        FSS_Together(i-7,3)-FSS_Together(i-8,3)...
        FSS_Together(i-7,4)-FSS_Together(i-8,4)...
        FSS_Together(i-7,5)-FSS_Together(i-8,5)...
        FSS_Together(i-7,6)-FSS_Together(i-8,6)...
        FSS_Together(i-7,7)-FSS_Together(i-8,7)...
        FSS_Together(i-7,8)-FSS_Together(i-8,8)...
        FSS_Together(i-7,9)-FSS_Together(i-8,9)...
        FSS_Together(i-7,10)-FSS_Together(i-8,10);...
       
        FSS_Together(i-8,1)-FSS_Together(i-9,1)...
        FSS_Together(i-8,2)-FSS_Together(i-9,2)...
        FSS_Together(i-8,3)-FSS_Together(i-9,3)...
        FSS_Together(i-8,4)-FSS_Together(i-9,4)...
        FSS_Together(i-8,5)-FSS_Together(i-9,5)...
        FSS_Together(i-8,6)-FSS_Together(i-9,6)...
        FSS_Together(i-8,7)-FSS_Together(i-9,7)...
        FSS_Together(i-8,8)-FSS_Together(i-9,8)...
        FSS_Together(i-8,9)-FSS_Together(i-9,9)...
        FSS_Together(i-8,10)-FSS_Together(i-9,10);...
        
        FSS_Together(i-9,1)-FSS_Together(i-10,1)...
        FSS_Together(i-9,2)-FSS_Together(i-10,2)...
        FSS_Together(i-9,3)-FSS_Together(i-10,3)...
        FSS_Together(i-9,4)-FSS_Together(i-10,4)...
        FSS_Together(i-9,5)-FSS_Together(i-10,5)...
        FSS_Together(i-9,6)-FSS_Together(i-10,6)...
        FSS_Together(i-9,7)-FSS_Together(i-10,7)...
        FSS_Together(i-9,8)-FSS_Together(i-10,8)...
        FSS_Together(i-9,9)-FSS_Together(i-10,9)...
        FSS_Together(i-9,10)-FSS_Together(i-10,10)];
        
        
    temp_3 = transpose(FFT_diff_3)*FFT_diff_3;
    Cov_3 = sum(temp_3(:)) - trace(temp_3);
    Cov_Array_3(i) = Cov_3;
    
end

for i=1500:2000
    Cov_Array_1(i) = Cov_Array_1(i)*15;
    Cov_Array_2(i) = Cov_Array_2(i)*15;
    Cov_Array_3(i) = Cov_Array_3(i)*15;
end

%% Plotting ArUco and FSS Sum
%{
figure(1)
set(gcf, 'Position', [100 0 2200 1000], 'color', 'white')
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
set(gcf, 'Position', [100 0 2200 1000], 'color', 'white')
%}

%% Plotting 4 Data Processing Options (Force Sum, Force Derivative, Cov and Frequency)
%{
figure(1)
set(gcf, 'Position', [100 -400 2000 1200], 'color', 'white')
plot(FSS_sum(1:2200), 'linewidth', 12)
xlabel('Timestep', 'FontSize', 32)
ylabel('Sum of Output Signal', 'FontSize', 20)
title('Sum of 10-Channel Output Signal from FSS Arrays', 'FontSize', 32)
ylim([-0.05, 0.23])
xlim([0 2250])
set(gca, 'FontSize', 32)
grid on

figure(2)
set(gcf, 'Position', [100 -400 2000 1200], 'color', 'white')
set(gca, 'FontSize', 32)
plot(Force_Derivative(1:2200), 'linewidth', 8, 'color', [0.8500 0.3250 0.0980])
xlabel('Timestep', 'FontSize', 32)
ylabel('Difference in Signal', 'FontSize', 20)
title('Difference in Sum Output Per Timestep', 'FontSize', 32)
grid on
xlim([0 2250])
set(gca, 'FontSize', 32)

figure(3)
set(gcf, 'Position', [100 -400 2000 1200], 'color', 'white')
set(gca, 'FontSize', 32)
plot(Cov(1:2200)*1000, 'linewidth', 8, 'color', [0.9290 0.6940 0.1250])
xlabel('Timestep', 'FontSize', 32)
ylabel('Covariance', 'FontSize', 20)
title('Covariance Among FSS Sensor Channels per Timestep', 'FontSize', 32)
grid on
ylim([-0.2 0.4])
xlim([0 2250])
set(gca, 'FontSize', 32)

Fs=80;

FSS_sum_array = FSS_sum(600-Fs:600);
FSS_sum_FFT = fft(FSS_sum_array);
P2 = abs(FSS_sum_FFT/Fs);
P1 = P2(1:Fs/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(Fs/2))/Fs;
f = f(2:end);
P1 = P1(2:end);
i1000_f = f;
i1000 = P1;

FSS_sum_array = FSS_sum(1500-Fs:1500);
FSS_sum_FFT = fft(FSS_sum_array);
P2 = abs(FSS_sum_FFT/Fs);
P1 = P2(1:Fs/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(Fs/2))/Fs;
f = f(2:end);
P1 = P1(2:end);
i1600_f = f;
i1600 = P1;

FSS_sum_array = FSS_sum(2100-Fs:2100);
FSS_sum_FFT = fft(FSS_sum_array);
P2 = abs(FSS_sum_FFT/Fs);
P1 = P2(1:Fs/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(Fs/2))/Fs;
f = f(2:end);
P1 = P1(2:end);
i2000_f = f;
i2000 = P1*0.04;

figure(4)
set(gcf, 'Position', [100 -400 2000 1200], 'color', 'white')
stem(i2000_f,i2000, 'LineWidth', 8)
grid on
hold on
set(gca, 'FontSize', 32)
stem(i1600_f,i1600, 'LineWidth', 8, 'color', [0.4660 0.6740 0.1880])
stem(i1000_f,i1000, 'LineWidth', 8, 'color', [0.4940 0.1840 0.5560]	)
ylabel('FFT Amplitude')
xlabel('Sample Frequency(Hz)')
ylim([0 0.0028])
xlim([0 41])
grid on
set(gca, 'FontSize', 14)
title('FFT Data of Sample Frequency 40Hz at 3 Different Phases', 'FontSize', 32)
legend('Severe Slip (Timestep 2100)', 'Mild Slip (Timestep 1600)', 'No Slip (Timestep 500)')
%}

%% Plotting Covariance for 3 Window Sizes
%{
figure(1)
set(gcf, 'Position', [2000 -1000 700 850], 'color', 'white')

subplot(3,1,1)
plot(Cov_Array_1(1:2200)*1000, 'color', [0.4940 0.1840 0.5560], 'linewidth', 2)
ylim([-0.18 0.28])
xlim([1400 2250])
grid on
hold on
title('Covariance of Data with Window Size n=1', 'FontSize', 16)
xlabel('Timestep', 'FontSize', 10)
ylabel('Covariance Amplitude', 'FontSize', 16)

subplot(3,1,2)
plot(Cov_Array_2(1:2200)*1000, 'color', [0.4940 0.1840 0.5560], 'linewidth', 2)
ylim([-0.18 0.28])
xlim([1400 2250])
grid on
hold on
title('Covariance of Data with Window Size n=5', 'FontSize', 16)
xlabel('Timestep', 'FontSize', 10)
ylabel('Covariance Amplitude', 'FontSize', 16)

subplot(3,1,3)
plot(Cov_Array_3(1:2200)*1000, 'color', [0.4940 0.1840 0.5560], 'linewidth', 2)
ylim([-0.18 0.28])
xlim([1400 2250])
grid on
hold on
title('Covariance of Data with Window Size n=10', 'FontSize', 16)
ylabel('Covariance Amplitude', 'FontSize', 16)
xlabel('Timestep', 'FontSize', 10)
%h = suptitle('Covariance of Data with Window Size n=1, n=5 and n=10');
%set(h, 'FontSize', 20)
%}

%% Plotting Frequency for n=10, 20 and 40
%{
for i = 1000:10:2250
    figure(1)
    set(gcf, 'Position', [2000 0 700 850], 'color', 'white')
    
    Fs = 20;
    FSS_sum_array = FSS_sum(i-Fs:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    
    subplot(3,1,1)
    stem(f,P1, 'LineWidth', 2.5, 'color', [0.8500 0.3250 0.0980])    
    xlim([0 11])
    ylim([0 0.005])
    grid on
    title('FFT Data of Sample Frequency 10Hz', 'FontSize', 16)
    ylabel('FFT Amplitude', 'FontSize', 16)
    xlabel('Sample Frequency(Hz)', 'FontSize', 10)
    if(i<1500)
        text(7.5, 0.004, 'No Slip', 'Color', 'k', 'FontSize', 24)
    elseif(i<1960)
        text(7.5, 0.004, 'Mild Slip', 'Color', 'red', 'FontSize', 24)
    elseif(i<2080)
        text(7.5, 0.004, 'Severe Slip', 'Color', 'red', 'FontSize', 24)
    else
        text(7.5, 0.004, 'No Slip', 'Color', 'k', 'FontSize', 24)
    end
    
    Fs_2 = 40;
    FSS_sum_array_2 = FSS_sum(i-Fs_2:i);
    FSS_sum_FFT_2 = fft(FSS_sum_array_2);
    P2_2 = abs(FSS_sum_FFT_2/Fs_2);
    P1_2 = P2_2(1:Fs_2/2+1);
    P1_2(2:end-1) = 2*P1_2(2:end-1);
    f_2 = Fs_2*(0:(Fs_2/2))/Fs_2;
    f_2 = f_2(2:end);
    P1_2 = P1_2(2:end);
    
    subplot(3,1,2)
    stem(f_2, P1_2, 'LineWidth', 2.5, 'color', [0.8500 0.3250 0.0980])
    xlim([0 21])
    ylim([0 0.005])
    grid on
    title('FFT Data of Sample Frequency 20Hz', 'FontSize', 16)
    ylabel('FFT Amplitude', 'FontSize', 16)
    xlabel('Sample Frequency(Hz)', 'FontSize', 10)
    
    Fs_3 = 80;
    FSS_sum_array_3 = FSS_sum(i-Fs_3:i);
    FSS_sum_FFT_3 = fft(FSS_sum_array_3);
    P2_3 = abs(FSS_sum_FFT_3/Fs_3);
    P1_3 = P2_3(1:Fs_3/2+1);
    P1_3(2:end-1) = 2*P1_3(2:end-1);
    f_3 = Fs_3*(0:(Fs_3/2))/Fs_3;
    f_3 = f_3(2:end);
    P1_3 = P1_3(2:end);
    
    subplot(3,1,3)
    stem(f_3, P1_3, 'LineWidth', 2.5, 'color', [0.8500 0.3250 0.0980])
    xlim([0 41])
    ylim([0 0.005])
    grid on
    title('FFT Data of Sample Frequency 40Hz', 'FontSize', 16)
    ylabel('FFT Amplitude', 'FontSize', 16)
    xlabel('Sample Frequency(Hz)', 'FontSize', 10)
        
    h = suptitle(['Timestep: ' num2str(i)])
    set(h, 'FontSize', 20)
    
    frame = getframe(1);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im,256);
    filename_ = 'FFT_10Hz_20Hz_40Hz.gif';
    del = 0.1;
    if(i == 1000)
      imwrite(imind,cm,filename_,'gif','Loopcount',inf,'DelayTime',del);
    else
      imwrite(imind,cm,filename_,'gif','WriteMode','append','DelayTime',del);
    end
   
end
%}

%% Plotting P-value Results for Data_Selection
%{
Derivative_Slip_Data = Force_Derivative(1500:2080);
Derivative_Non_Slip_Data = Force_Derivative(1:1500);
Covariance_Slip_Data = Cov_Array_3(1500:2080);
Covariance_Non_Slip_Data = Cov_Array_3(1:1500);

FFT_Non_Slip_Data = [];
k=1;
for i = 81:1500
    Fs = 80;
    FSS_sum_array = FSS_sum(i-Fs:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    FFT_Non_Slip_Data(k,:) = P1;
    k=k+1;
end

FTT_Slip_Data = [];
k=1;
for i = 1501:2080
    Fs = 80;
    FSS_sum_array = FSS_sum(i-Fs:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    FFT_Slip_Data(k,:) = P1;
    k=k+1;
end

h_FFT_value=[];
p_FFT_value = [];
for i=1:Fs/2
    FFT_Slip_Data_ = FFT_Slip_Data(:,i);
    FFT_Non_Slip_Data_ = FFT_Non_Slip_Data(:,i);
    
    [h_FFT, p_FFT] = kstest2(FFT_Slip_Data_, FFT_Non_Slip_Data_, 'Alpha', 0.05);
        
    h_FFT_value(i) = h_FFT;
    p_FFT_value(i) = p_FFT;
end

h_FFT = sum(h_FFT_value)/(Fs/2);
p_FFT = sum(p_FFT_value)/(Fs/2);

[h_Derivative, p_Derivative] = kstest2(Derivative_Slip_Data, Derivative_Non_Slip_Data, 'Alpha', 0.05);
[h_Covariance, p_Covariance] = kstest2(Covariance_Slip_Data, Covariance_Non_Slip_Data, 'Alpha', 0.05);

figure(1)
set(gcf, 'Position', [500 0 1000 800], 'color', 'white')
Legend = {'Force Sum', 'Force Derivative', 'Covariance', 'FFT Data (Avg)'};
X = [h_Derivative, h_Covariance, h_FFT];
Y = [p_Derivative, p_Covariance, p_FFT];

bar(Y)
set(gca, 'xticklabel', Legend, 'FontSize', 16)
grid on
title('Bar Graph of the P-Values of Each Processed Data', 'FontSize', 20)
ylabel('P-Value', 'FontSize', 16)
%}

%% Plotting Confusion Matrix
%{
Accuracy = readmatrix('x/Accuracy_500_Epoch.csv');
figure(1)
set(gcf, 'Position', [100 100 1000 600], 'color', 'white')
plot(Accuracy(:,2), 'LineWidth', 8)
grid on
set(gca, 'FontSize', 32)
title('Training Accuracy', 'FontSize', 32)
ylim([-0.03 1.05])
xlim([-10 505])
ylabel('Accuracy')
xlabel('Epoch')

Online_Data = readmatrix('Online_Data.csv');
Ground_Truth = Online_Data(:,1);
Prediction = Online_Data(:,2);
figure(2)
set(gcf, 'Position', [100 0 1200 800], 'color', 'white')
C = confusionmat(Ground_Truth, Prediction);
label =  {'No Slip', 'Slip'};
label = categorical(label);
confusionchart(C, label);
ylabel('Ground Truth');
xlabel('Prediction');
set(gca, 'FontSize', 40)


Online_Data = readmatrix('Online_Data_2.csv');
Ground_Truth = Online_Data(:,1);
Prediction = Online_Data(:,2);
figure(3)
set(gcf, 'Position', [100 0 1200 800], 'color', 'white')
C = confusionmat(Ground_Truth, Prediction);
label =  {'No Slip', 'Slip'};
label = categorical(label);
confusionchart(C, label);
ylabel('Ground Truth');
xlabel('Prediction');
set(gca, 'FontSize', 40)
title('Confusion Matrix of Network With Lookback', 'FontSize', 32);
%}

%% Computing FSS for Data Examination Purposes
%{
Processed_Data = readmatrix('x/Processed_Data.csv');
Processed_Data(:,1)=[];
Processed_Data(1,:)=[];

Fs = 80;
for i=2000:2200
    FSS_sum_array = FSS_sum(i-Fs+1:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    figure(1)
    set(gcf, 'Position', [50 50 1600 850], 'color', 'white')
    stem(f, P1, 'LineWidth', 8)
    ylim([0 0.01])
    title(i)
    pause(0.1)
end
%}


