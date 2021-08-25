function[t_data, data] = topic_read(bag, Topic_name, Message_name)

temp = select(bag, 'Topic', Topic_name);
read_temp = readMessages(temp, 'DataFormat', 'struct');

for i = 1:length(read_temp)
    data(i,:) = read_temp{i}.(Message_name);
    t_data(i,:) = temp.MessageList.Time(i);
end
