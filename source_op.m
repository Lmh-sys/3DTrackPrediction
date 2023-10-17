% 功能说明：ekf,ukf,pf,改进pf算法的无人机航迹预测比较程序

function source_op

    clc; clear; close all;
    % 因本程序涉及太多的随机数，下面让随机数每次都不变
    rand('seed',3);
    randn('seed',6);
    % error('下面的参数T请参考书中的值设置，然后删除本行代码') 
    dim = 3;
    T = 50; % 模拟长度
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % 系统初始化
    N = 200;         % 粒子滤波粒子数    
    real_source = zeros(dim,T);  % 真实值
    measurement_stacked = zeros(1,T);  % 观测值
    % 真实状态初始化
    mapRange=150;
    Station=ones(2,T);   % 观测站的位置
    real_source(:,1)=[mapRange*0.9, mapRange*0.6, 2.94e8]'+sqrtm(Q)*randn(dim,1); % 正式寻源，放射源参数向量，用于构建辐射场，生成观测向量集
    measurement_stacked(:,1) = underlying_model(Station(1:2,1),real_source(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF滤波算法 %%%%%%%%%%%%
    Xpf=zeros(dim,T);        % 滤波状态估计
    Xmap_pf = zeros(3, T);             % PF滤波最大似然预测
    Xpf(:,1)=real_source(:,1);       % PF滤波预测
    Xmap_pf(:,1)=real_source(:,1);       
    Xpfset=ones(dim,N);      % 粒子集合初始化
    Tpf=zeros(1,T);        % 用于记录一个采样周期的算法时间消耗

    for j=1:N   % 粒子集初始化
        Xpfset(1, j) = unifrnd(0,mapRange);
        Xpfset(2, j) = unifrnd(0,mapRange);
        % Xpfset(3, j) = 2.94e8;
        Xpfset(3, j) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    end
    %%%%%%%%%%%%% PSO 粒子群 %%%%%%%%%%%%
    nvars = 2;
    Wmax = 0.9; % 惯性系数
    c1 = 1.4955; %学习因子
    c2 = 1.4955; %学习因子
    max_iter = 100;
    v_limit = 30;
    vel = -v_limit+v_limit*2*rand(nvars,N);
    [gbest, pbest, Xpfset] = pso_init(@GetFitness, Station(:,1), measurement_stacked(:,1), Xpfset);
    %%%%%%%%%%%%%%%%%%%%%% 模拟系统运行 %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 初始化场景显示
    [X,Y,Z] = defMap(mapRange);
    init_map(X,Y,Z,Xpfset,Station,real_source)
    
    stop = T;
    for t=2:T
        if norm(Station(1:2,t -1) - real_source(1:2,t -1)) < mapRange/15
            stop = t-1;
            break;
        end
        % 模拟系统状态运行一步
        Station(1:2,t) = Station(1:2,t-1) + 0.2*(Xpf(1:2,t-1) - Station(1:2,t-1));
        real_source(:,t) = real_source(:,t-1);  % 产生实际状态值
        measurement_stacked(:,t) = underlying_model(Station(1:2,t),real_source(:,t)) + sqrtm(R)*randn(1,1);
        [gbest, pbest, pose,vel] = pso(@GetFitness, 0, mapRange, max_iter,N, Wmax, c1, c2, Station(1:2,t),  measurement_stacked(:,t), Xpfset, vel, pbest, gbest, v_limit);
        Xpfset(1:2,:) = pose;
        tic
        [Xpf(:,t),Xpfset,Neffpf]=pff(Xpfset,measurement_stacked(:,t),N,dim,R,Q,Station(1:2,t));
        Tpf(t)=toc;
        for idex= 1:3
            [p, pos] = hist(Xpfset(idex, t), 20,2);
            map = find(p == max(p));
            Xmap_pf(idex, t) = pos(map(1));
        end
        Xpf(:,t) = Xmap_pf(:,t);
        if isnan(Neffpf)
            disp("12312");
        end
        map_show(X,Y,Z,Xpfset,Station,real_source,t,mapRange,Xmap_pf)
    end
    %%%%%%%%%%%%%%%%%%%%%% 数据分析 %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % X轴RMS偏差比较图
    PFXrms = zeros(1,stop);
    PFYrms = zeros(1,stop);
    PFZrms = zeros(1,stop);
    for t=1:stop
        PFXrms(1,t)=abs(real_source(1,t)-Xpf(1,t))/real_source(1,t);
        PFYrms(1,t)=abs(real_source(2,t)-Xpf(2,t))/real_source(2,t);
        PFZrms(1,t)=abs(real_source(3,t)-Xpf(3,t))/real_source(3,t);
    end
    
    figure
    t=1:stop;
    hold on;
    box on;
    grid on;
    p1 = plot3(real_source(1,t),real_source(2,t),real_source(3,t),'-k.','lineWidth',1);
    % $$ r= \sqrt{x^{2}+y^{2}+z^{2}} $$,
    % $$  \theta = \arccos(\frac{\sqrt{x^{2}+y^{2}}}{r})= \arctan(\frac{\sqrt{x^{2}+y^{2}}}{r}) $$
    % $$  \varphi = \arccos(\frac{y}{r \sin \theta})= \arctan(\frac{y}{x}). $$
    p2 = plot3(Station(1,t),Station(2,t),measurement_stacked(1,t),'m:','lineWidth',2);
    p5 = plot3(Xpf(1,t),Xpf(2,t),Xpf(3,t),'-g*','lineWidth',1);
    legend([p1,p2,p5],'真实状态','观测状态','PF估计');
    xlabel('x轴位置');
    ylabel('y轴位置');
    zlabel('z轴位置');
    axis([0, mapRange, 0, mapRange]);
    view(3);
    
    figure
    hold on;
    box on;
    p1=plot(1:stop,PFXrms,'-k.','lineWidth',2);
    p2=plot(1:stop,PFYrms,'-m^','lineWidth',2);
    p3=plot(1:stop,PFZrms,'-ro','lineWidth',2);
    legend([p1,p2,p3],'PF-X偏差','PF-Y偏差','PF-Z偏差');
    xlabel('time step');
    ylabel('RMS预测偏差');
    
    figure;
    hold on;
    box on;
    p3=plot(1:stop,Tpf(1:stop),'-ro','lineWidth',2);
    legend([p3],'PF时间');
    xlabel('time step');
    ylabel('单步时间/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 再画一个不同Q、R得到的不同的结果图
    % 再画一个偏差曲线图
    
    norm(real_source(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    