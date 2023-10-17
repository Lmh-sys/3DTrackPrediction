% 功能说明：ekf,ukf,pf,改进pf算法的无人机航迹预测比较程序

function source

    clc; clear; close all;
    % 因本程序涉及太多的随机数，下面让随机数每次都不变
    % error('下面的参数T请参考书中的值设置，然后删除本行代码') 
    dim = 3;
    T = 50; % 模拟长度
    
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % 系统初始化
    N = 200;         % 粒子滤波粒子数    
    X = zeros(dim,T);  % 真实值
    Z = zeros(1,T);  % 观测值
    % 真实状态初始化
    range=150;
    step = range/1500;
    Station=zeros(2,T);   % 观测站的位置
    X(:,1)=[range*0.9, range*0.6, 2.94e8]'+sqrtm(Q)*randn(dim,1); % 正式寻源，放射源参数向量，用于构建辐射场，生成观测向量集
    Z(:,1) = underlying_model(Station(1:2,1),X(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF滤波算法 %%%%%%%%%%%%
    Xpf=zeros(dim,T);        % 滤波状态估计
    Xpf(:,1)=X(:,1);       % PF滤波预测
    Xpfset=ones(dim,N);      % 粒子集合初始化
    Tpf=zeros(1,T);        % 用于记录一个采样周期的算法时间消耗
    rand('seed',3);
    randn('seed',6);
    for j=1:N   % 粒子集初始化
        Xpfset(1, j) = unifrnd(0,range);
        Xpfset(2, j) = unifrnd(0,range);
        % Xpfset(3, j) = 2.94e8;
        Xpfset(3, j) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    end
    %%%%%%%%%%%%% PSO 粒子群 %%%%%%%%%%%%
    nn = N; %粒子群规模，即粒子数 %粒子的维数，即寻优参数的个数
    pso_dim = 6;
    Wmax = 0.9; % 惯性系数
    Wmin = 0.5;
    iter= 1;
    ITERmax = 100; %进化代数
    c1 = 1.4955; %学习因子
    c2 = 1.4955; %学习因子
    pose_limit = range; % 位置限制
    alpha = 1;%约束因子=控制速度的权重
    v_limit = 30;%速度限制

    X_pso = zeros(pso_dim,nn); %初始化粒子群[编号 位置x 位置y 活度I 速度x 速度y]
    Pbest = zeros(4,nn); %个体历史最优 每行=[编号 位置x 位置y 活度I] 
    Gbest = zeros(3,1); %群体历史最优 每行=[位置x 位置y 活度I] (整个粒子群仅有一个)
    pres_Pbest = zeros(1,nn);
    Gbest(1) = pose_limit*rand(1,1);% 全局最优初值
    Gbest(2) = pose_limit*rand(1,1);% 全局最优初值
    Gbest(3) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    vel = -v_limit+v_limit*2*rand(2,N);
    %赋予粒子群初值(每个粒子随机位置和速度)
    for i = 1: 1: nn
        %每个粒子
        X_pso(1,i) = i;%编号
        X_pso(2,i) = Xpfset(1,i);
        X_pso(3,i) = Xpfset(2,i);
        X_pso(4,i) = Xpfset(3,i);
        X_pso(5,i) = vel(1,i);%vx=-v_limit~v_limit
        X_pso(6,i) = vel(2,i);%vy=-v_limit~v_limit
        %个体历史最优
        Pbest(1,i) = X_pso(1,i);%编号
        Pbest(2,i) = X_pso(2,i);%x
        Pbest(3,i) = X_pso(3,i);%y
        Pbest(4,i) = X_pso(4,i);%y
        %群体历史最优初值
        w_pbest = GetFitness(Station(1:2,1), Pbest(2:4,i), Z(:,1));
        pres_Pbest(i) = w_pbest;
        w_gbest = GetFitness(Station(1:2,1), Gbest, Z(:,1));
        if w_pbest > w_gbest
            Gbest = Pbest(2:4,i);
        end
    end
    % [gbest, gbestval] = psoff(@GetFitness, pso_dim, 0, range, 100, pop_size,0.9, c1, c2);
    %%%%%%%%%%%%%%%%%%%%%% 模拟系统运行 %%%%%%%%%%%%%%%%%%%%%%%%%
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    % 初始化场景显示
    figure(1);hold off;
    plot(X_pso(2, :), X_pso(3, :),'.','Color',robot_color(1),'markersize',4);hold on
    plot(Station(1,1),Station(2,1),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
    plot(X(1,1),X(2,1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
    bins = 20;
    Xmap_pf = zeros(3, T);
    Xmap_pf(:,1)=X(:,1);       % PF滤波预测
    for t=2:T
        if norm(Station(1:2,t -1) - X(1:2,t -1)) < range/15
            break;
        end
        % 模拟系统状态运行一步
        Station(1:2,t) = Station(1:2,t-1) + 0.2*(Xpf(1:2,t-1) - Station(1:2,t-1));
        X(:,t) = X(:,t-1) + sqrtm(Q) * randn(dim,1);  % 产生实际状态值
        Z(:,t) = underlying_model(Station(1:2,t),X(:,t)) + sqrtm(R)*randn(1,1);
        % 粒子群迭代
        while (iter < ITERmax)
            for i = 1: 1: nn%每个粒子
                v_id = X_pso(5:6,i);%i粒子的速度=(v_x v_y)
                x_id = X_pso(2:3,i);%i粒子的位置=(x_x x_y)
                p_id = Pbest(2:3,i);%i粒子的个体历史最优位置
                r1 = rand(1,1);%r1,r2是介于[0,1]之间的随机数
                r2 = rand(1,1);
                % W_pso=Wmax-(Wmax-Wmin)/ITERmax*iter; %惯性因子
                W_pso = Wmin + (Wmax - Wmin) * exp(-3 * (iter / ITERmax) .^ 2);
                %  W_pso=(W_pso1+W_pso2)/2;

                v_id = W_pso*v_id + c1*r1*(p_id-x_id) + c2*r2*(Gbest(1:2)-x_id);
                v_id( v_id> v_limit ) = v_limit;
                v_id( v_id< -v_limit ) = -v_limit; 
                x_id = x_id + alpha*v_id;
                %2) 对于越界的粒子进行调整: 反射边界上=速度大小不变 方向取反
                if ( x_id(1)>range && v_id(1)>0 ) || ( x_id(1)< 0  && v_id(1)<0 )%x+或x-方向越界
                    x_id = x_id - alpha*v_id;%恢复之前的位置
                    v_id(1) = -v_id(1);%x方向上速度取反
                    x_id = x_id + alpha*v_id;%然后再运动
                end
                if ( x_id(2)> range && v_id(2)>0 ) || ( x_id(2)< 0 && v_id(2)<0 )%y+或y-方向越界
                    x_id = x_id - alpha*v_id;%恢复之前的位置
                    v_id(2) = -v_id(2);%y方向上速度取反
                    x_id = x_id + alpha*v_id;%然后再运动
                end
                %3) 粒子运动
                %刷新粒子的位置和速度
                X_pso(2:3,i) = x_id;
                X_pso(5:6,i) = v_id;
                w_partices = GetFitness(Station(1:2,t), X_pso(2:4,i), Z(:,t));
                w_pbest = GetFitness(Station(1:2,t), Pbest(2:4,i), Z(:,t));
                pres_Pbest(i) = w_pbest;
                w_gbest = GetFitness(Station(1:2,t), Gbest, Z(:,t));
                %刷新个体历史最优
                if w_partices > w_pbest
                    Pbest(2:4,i) = X_pso(2:4,i);
                end
                %刷新群体历史最优
                if w_partices > w_gbest
                    Gbest = X_pso(2:4,i);%tmp_pg 不影响这一轮后续的粒子判断
                end
            end
            % 可视化
            figure(1);hold off;
            plot(X_pso(2, :), X_pso(3, :),'.','Color',robot_color(1),'markersize',4);hold on
            plot(Station(1,t),Station(2,t),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
            plot(X(1,1),X(2,1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
            plot(Xmap_pf(1,t-1),Xmap_pf(2,t-1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
            axis([0, range, 0, range]);
            if w_pbest >= w_gbest*0.9 %粒子群最优值符合某个阀值的时候，退出循环，认为粒子已经分布在真实状态附近。
                iter = ITERmax;
            else
                iter = iter + 1;
            end
        end %粒子群循环结束
        iter = 1 ;
        tic
        % [Xpf(:,t),Xpfset,Neffpf]=pff(Xpfset,Z(:,t),N,dim,R,Q,Station(1:2,t));        
        [Xpf(:,t),X_pso(2:4,:),Neffpf]=pff(X_pso(2:4,:),Z(:,t),N,dim,R,Q,Station(1:2,t));                             % 搞定
        Tpf(t)=toc;
        for idex= 2:4
            [p, pos] = hist(X_pso(idex, t), bins,2);
            map = find(p == max(p));
            Xmap_pf(idex-1, t) = pos(map(1));
        end
        Xpf(:,t) = Xmap_pf(:,t);
        if isnan(Neffpf)
            disp("12312");
        end
        % if Neffpf > 2*N /3
        %     iter =1;
        %     % var_x = 100*step/t; % 粒子基于正态分布采样时的位置的方差var_p = 50,100 
        %     % var_y = 100*step/t;
        %     % % var_I = 4.5e7*t; % 粒子基于正态分布采样时的强度的方差
        %     % X_pso(2,:) = normrnd(X_pso(2,:), var_x);
        %     % X_pso(3,:) = normrnd(X_pso(3,:), var_y);
        %     % % Xpfset(3,:) = random('poisson',mean(Xpfset(3,:)),1,200);
        % end

        figure(1);hold off;
        plot(Station(1,t),Station(2,t),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
        plot(X_pso(2, :), X_pso(3, :),'.','Color',robot_color(1),'markersize',4);hold on
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
    
    % figure;
    % hold on;
    % box on;
    % p3=plot(1:T,Tpf,'-ro','lineWidth',2);
    % legend([p3],'PF时间');
    % xlabel('time step');
    % ylabel('单步时间/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 再画一个不同Q、R得到的不同的结果图
    % 再画一个偏差曲线图
    
    norm(X(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    