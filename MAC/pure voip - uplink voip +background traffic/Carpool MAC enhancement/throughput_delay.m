clc;close all;clear all;

for num = 10:4:30
str = ['2ap',num2str(num),'client_VOIP.mat'];
load(str);
disp(str);
disp(sum(cellfun(@sum,SentPktCount_List(1:2)))/(DATA_Rate*time));
sumLen = length(Traffic_delay_List{1}) + length(Traffic_delay_List{2});
disp((sum(Traffic_delay_List{1}(1:end))+sum(Traffic_delay_List{2}(1:end)))/sumLen);
end
