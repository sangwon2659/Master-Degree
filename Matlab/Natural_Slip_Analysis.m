%% Sensor system validation
clear all
close all
clc
t_step=0.005;

%% Bag Read
varname = strings;
filename = "rosbag/2021-08-05-10-45-05.bag";
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

%{
%%
Data_i.t_HCmotor = [];
Data_i.HCmotor = [];
i=1;

while (t_range(i)<Data.t_HCmotor(1))
    i=i+1;
end

Data_i.Slip_Vector = Data_i.Slip_Vector(i:end,:);
Data_i.FSS;
m=1;

for k=1:length(Data_i.Slip_Vector)-1
    Data_i.t_HCmotor(k) = t_range(i+k);
    if(m+1<length(Data.HCmotor))
        if(t_range(i+k)>Data.t_HCmotor(m+1))
            m=m+1;
        end
    end
    Data_i.HCmotor(k) = Data.HCmotor(m);
end

for j = 1:length(Data_i.HCmotor)
    if(Data_i.HCmotor(j)~=0)
        Data_i.HCmotor(j) = 1;
    else
        Data_i.HCmotor(j) = 0;
    end        
end
%}


%% Figure
for i = 2:length(Data_i.FSS)
    disp(i)
    disp(sum(Data_i.FSS(i,:)))
    if(abs(sum(Data_i.FSS(i,:))) < 5)
        FSS_Together(i) = sum(Data_i.FSS(i,:));
    else
        FSS_Together(i) = 0;
    end
end

figure(1)
plot(FSS_Together)

Initial = 1;
Final = length(Data_i.Slip_Vector);

%{
for i = Initial:Final
    
    figure(2)
    set(gcf, 'Position', [0 50 900 400], 'color', 'white')
    stem(Data_i.Slip_Vector(i,:))
    grid on
    title(i)
    ylim([0 0.08])
    pause(0.001)
    
end
%}


























