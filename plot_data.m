clear all
clc
close all

% desktop
% cd '/home/chad/Documents/mujoco/projects/Quad_Template/data'
% labtop 

cd '/home/chad/Documents/mujoco-2.2.1/myProject/Quad_Template/data'
filename = '/data_FL.csv';

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
plot(Arr(500:m,1),Arr(500:m,2), 'r-' ,Arr(500:m,1), Arr(500:m,3), 'b-');
ylabel(cell2mat(VariableNames(2)))
legend('dist_reff','d_hat')
% 
% figure(2);
% plot(Arr(500:m,1),Arr(500:m,4), 'r-' ,Arr(500:m,1), Arr(500:m,5), 'b-',Arr(500:m,1), Arr(500:m,6), 'g-');
% % ylabel()
% legend('pos','vel', 'acc')
% 
% figure(3);
% plot(Arr(500:m,1),Arr(500:m,7), 'r-' , Arr(500:m,1),Arr(500:m,8), 'b-' ,Arr(500:m,1),Arr(500:m,9), 'g-');
% % ylabel()
% legend('dhat' , 'dist','off_dist')
