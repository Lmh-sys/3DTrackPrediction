%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �۲ⷽ�̺���
 function [dd,alpha,beta]=hfun(X1,Station)
if nargin < 2
    error('Not enough input arguments.');
end

% ��������ϵתֱ������ϵ�� $(r, \theta, \phi)$ �ֱ��ʾ���롢���Ǻͷ�λ�ǡ����У�$\theta$ ��ʾ��������ļнǣ�$\phi$ ��ʾ�� $xy$ ƽ���ϵ�ͶӰ���� $x$ ��ļнǡ�
% https://zh.wikipedia.org/zh-hans/%E7%90%83%E5%BA%A7%E6%A8%99%E7%B3%BB
dd = sqrt((X1(1,1)-Station(1,1))^2+(X1(2,1)-Station(2,1))^2+(X1(3,1)-Station(3,1))^2);
alpha = atan((X1(2,1)-Station(2,1))/(X1(1,1)-Station(1,1))); % ƫ���� yaw (z)
beta = atan((X1(3,1)-Station(3,1))/sqrt((X1(1,1)-Station(1,1))^2+(X1(2,1)-Station(2,1))^2)); %������ pitch (y)

