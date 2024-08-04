clear all
clc
close all

% desktop
%cd '/home/cha/Documents/mujoco/projects/chad_simulator/data'
% labtop 
cd '/home/chad/Documents/mujoco/myProject/Quad_Template4/data'
filename = '/data.csv';


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

% figure(1);
% plot(Arr(500:m,1),Arr(500:m,2), 'r-' ,Arr(500:m,1), Arr(500:m,3), 'b-');
% ylabel(cell2mat(VariableNames(2)))
% legend('ref','FL_hip')
% 
% figure(2);
% plot(Arr(500:m,1),Arr(500:m,4), 'r-' ,Arr(500:m,1), Arr(500:m,5), 'b-',Arr(500:m,1), Arr(500:m,6), 'g-');
% % ylabel()
% legend('pos','vel', 'acc')

figure(3);
plot(Arr(1:m,1),Arr(1:m,2), 'r-' , Arr(1:m,1),Arr(1:m,4), 'b-' );
legend('ref','curr')

figure(4);
plot(Arr(1:m,1),Arr(1:m,3), 'r-' ,Arr(1:m,1),Arr(1:m,5), 'b-' );
legend('ref','curr')
% ylabel()
