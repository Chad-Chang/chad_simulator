clear all
clc
close all

% desktop
% cd '/home/chad/Documents/mujoco/projects/chad_simulator/data'
% labtop 
cd '/home/chad_chang/mujoco/myProject/chad_simulator/data'
filename = '/ROBOT_DOB.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

% for i=2:n
%     figure(i)
%     yy = i;
%     plot(Arr(:,yy),'r');
%     ylabel(cell2mat(VariableNames(yy)))
% %     yy = i;
% %     plot(Arr(:,1),Arr(:,yy),'r');
% %     ylabel(cell2mat(VariableNames(yy)))
% %     xlabel(cell2mat(VariableNames(1)))
% end

figure(1);
plot(Arr(5000:m,1),Arr(5000:m,2), 'r-' ,Arr(5000:m,1), Arr(5000:m,4), 'b-');
ylabel(cell2mat(VariableNames(2)))
legend('ref','FL_hip')
