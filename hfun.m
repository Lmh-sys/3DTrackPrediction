%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 观测方程函数
 function [dd,alpha,beta]=hfun(X1,Station)
if nargin < 2
    error('Not enough input arguments.');
end

% 在球坐标系转直角坐标系， $(r, \theta, \phi)$ 分别表示距离、极角和方位角。其中，$\theta$ 表示与正极轴的夹角，$\phi$ 表示在 $xy$ 平面上的投影与正 $x$ 轴的夹角。
% https://zh.wikipedia.org/zh-hans/%E7%90%83%E5%BA%A7%E6%A8%99%E7%B3%BB
dd = sqrt((X1(1,1)-Station(1,1))^2+(X1(2,1)-Station(2,1))^2+(X1(3,1)-Station(3,1))^2);
alpha = atan((X1(2,1)-Station(2,1))/(X1(1,1)-Station(1,1))); % 偏航角 yaw (z)
beta = atan((X1(3,1)-Station(3,1))/sqrt((X1(1,1)-Station(1,1))^2+(X1(2,1)-Station(2,1))^2)); %俯仰角 pitch (y)

