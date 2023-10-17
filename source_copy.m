% 功能说明：ekf,ukf,pf,改进pf算法的无人机航迹预测比较程序

function source

    clc; clear; close all;
    % 因本程序涉及太多的随机数，下面让随机数每次都不变
    % error('下面的参数T请参考书中的值设置，然后删除本行代码') 
    n = 3;
    T = 50; % 模拟长度
    
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % 系统初始化
    N = 200;         % 粒子滤波粒子数    
    X = zeros(n,T);  % 真实值
    Z = zeros(1,T);  % 观测值
    % 真实状态初始化
    range=150;
    step = range/1500;
    Station=zeros(2,T);   % 观测站的位置
    X(:,1)=[range*0.9, range*0.6, 2.94e8]'+sqrtm(Q)*randn(n,1); % 正式寻源，放射源参数向量，用于构建辐射场，生成观测向量集
    Z(:,1) = underlying_model(Station(1:2,1),X(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF滤波算法 %%%%%%%%%%%%
    Xpf=zeros(n,T);        % 滤波状态
    Xpf(:,1)=X(:,1);       % PF滤波预测
    Xpfset=ones(n,N);      % 粒子集合初始化
    Tpf=zeros(1,T);        % 用于记录一个采样周期的算法时间消耗

    %%%%%%%%%%%%%%%%%%%%%% 模拟系统运行 %%%%%%%%%%%%%%%%%%%%%%%%%
    for j=1:N   % 粒子集初始化
        Xpfset(1, j) = unifrnd(0,range);
        Xpfset(2, j) = unifrnd(0,range);
        % Xpfset(3, j) = 2.94e8;
        Xpfset(3, j) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    end
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    figure(1);hold off;
    plot(Xpfset(1, :), Xpfset(2, :),'.','Color',robot_color(1),'markersize',4);hold on
    plot(Xpf(1,1),Xpf(2,1),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
    plot(X(1,1),X(2,1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
    bins = 20;
    Xmap_pf = zeros(3, T);
    Xmap_pf(:,1)=X(:,1);       % PF滤波预测
    for t=2:T
        % 模拟系统状态运行一步
        Station(1:2,t) = Station(1:2,t-1) + step*(Xmap_pf(1:2,t-1) - Station(1:2,t-1));
        X(:,t) = X(:,t-1) + sqrtm(Q) * randn(n,1);  % 产生实际状态值
        Z(:,t) = underlying_model(Station(1:2,t),X(:,t)) + sqrtm(R)*randn(1,1);
        tic
        [Xpf(:,t),Xpfset,Neffpf]=pff(Xpfset,Z(:,t),N,n,R,Q,Station(1:2,t));                             % 搞定
        Tpf(t)=toc;
        for idex= 1:3
            [p, pos] = hist(Xpfset(idex, t), bins,2);
            map = find(p == max(p));
            Xmap_pf(idex, t) = pos(map(1));
        end
        Xpf(:,t) = Xmap_pf(:,t);
        

        if Neffpf < 10
            var_x = 100*step/t; % 粒子基于正态分布采样时的位置的方差var_p = 50,100 
            var_y = 100*step/t;
            % var_I = 4.5e7*t; % 粒子基于正态分布采样时的强度的方差
            Xpfset(1,:) = normrnd(Xpfset(1,:), var_x);
            Xpfset(2,:) = normrnd(Xpfset(2,:), var_y);
            % Xpfset(3,:) = random('poisson',mean(Xpfset(3,:)),1,200);
        end

        figure(1);hold off;
        plot(Station(1,t-1),Station(2,t-1),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
        plot(Xpfset(1, :), Xpfset(2, :),'.','Color',robot_color(1),'markersize',4);hold on
        plot(Xpf(1,t),Xpf(2,t),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
        plot(Xmap_pf(1,t),Xmap_pf(2,t),'o','MarkerFaceColor',robot_color(5),'markersize',4);hold on
        plot(X(1,t),X(2,t),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
        axis([0, range, 0, range]);
        drawnow limitrate;
        hold off
    end
    
    %%%%%%%%%%%%%%%%%%%%%% 数据分析 %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % X轴RMS偏差比较图
    PFXrms = zeros(1,T);
    PFYrms = zeros(1,T);
    PFZrms = zeros(1,T);
    for t=1:T
        PFXrms(1,t)=abs(X(1,t)-Xpf(1,t))/X(1,t);
        PFYrms(1,t)=abs(X(2,t)-Xpf(2,t))/X(2,t);
        PFZrms(1,t)=abs(X(3,t)-Xpf(3,t))/X(3,t);
    end
    
    figure
    t=1:T;
    hold on;
    box on;
    grid on;
    p1 = plot3(X(1,t),X(2,t),X(3,t),'-k.','lineWidth',1);
    % $$ r= \sqrt{x^{2}+y^{2}+z^{2}} $$,
    % $$  \theta = \arccos(\frac{\sqrt{x^{2}+y^{2}}}{r})= \arctan(\frac{\sqrt{x^{2}+y^{2}}}{r}) $$
    % $$  \varphi = \arccos(\frac{y}{r \sin \theta})= \arctan(\frac{y}{x}). $$
    p2 = plot3(Station(1,t),Station(2,t),Z(1,t),'m:','lineWidth',2);
    p5 = plot3(Xpf(1,t),Xpf(2,t),Xpf(3,t),'-g*','lineWidth',1);
    legend([p1,p2,p5],'真实状态','观测状态','PF估计');
    xlabel('x轴位置');
    ylabel('y轴位置');
    zlabel('z轴位置');
    axis([0, range, 0, range]);
    view(3);
    
    figure
    hold on;
    box on;
    p1=plot(1:T,PFXrms,'-k.','lineWidth',2);
    p2=plot(1:T,PFYrms,'-m^','lineWidth',2);
    p3=plot(1:T,PFZrms,'-ro','lineWidth',2);
    legend([p1,p2,p3],'PF-X偏差','PF-Y偏差','PF-Z偏差');
    xlabel('time step');
    ylabel('RMS预测偏差');
    
    figure;
    hold on;
    box on;
    p3=plot(1:T,Tpf,'-ro','lineWidth',2);
    legend([p3],'PF时间');
    xlabel('time step');
    ylabel('单步时间/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 再画一个不同Q、R得到的不同的结果图
    % 再画一个偏差曲线图
    
    norm(X(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    