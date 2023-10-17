% ����˵����ekf,ukf,pf,�Ľ�pf�㷨�����˻�����Ԥ��Ƚϳ���

function source_op

    clc; clear; close all;
    % �򱾳����漰̫���������������������ÿ�ζ�����
    rand('seed',3);
    randn('seed',6);
    % error('����Ĳ���T��ο����е�ֵ���ã�Ȼ��ɾ�����д���') 
    dim = 3;
    T = 50; % ģ�ⳤ��
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % ϵͳ��ʼ��
    N = 200;         % �����˲�������    
    real_source = zeros(dim,T);  % ��ʵֵ
    measurement_stacked = zeros(1,T);  % �۲�ֵ
    % ��ʵ״̬��ʼ��
    mapRange=150;
    Station=ones(2,T);   % �۲�վ��λ��
    real_source(:,1)=[mapRange*0.9, mapRange*0.6, 2.94e8]'+sqrtm(Q)*randn(dim,1); % ��ʽѰԴ������Դ�������������ڹ������䳡�����ɹ۲�������
    measurement_stacked(:,1) = underlying_model(Station(1:2,1),real_source(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF�˲��㷨 %%%%%%%%%%%%
    Xpf=zeros(dim,T);        % �˲�״̬����
    Xmap_pf = zeros(3, T);             % PF�˲������ȻԤ��
    Xpf(:,1)=real_source(:,1);       % PF�˲�Ԥ��
    Xmap_pf(:,1)=real_source(:,1);       
    Xpfset=ones(dim,N);      % ���Ӽ��ϳ�ʼ��
    Tpf=zeros(1,T);        % ���ڼ�¼һ���������ڵ��㷨ʱ������

    for j=1:N   % ���Ӽ���ʼ��
        Xpfset(1, j) = unifrnd(0,mapRange);
        Xpfset(2, j) = unifrnd(0,mapRange);
        % Xpfset(3, j) = 2.94e8;
        Xpfset(3, j) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    end
    %%%%%%%%%%%%% PSO ����Ⱥ %%%%%%%%%%%%
    nvars = 2;
    Wmax = 0.9; % ����ϵ��
    c1 = 1.4955; %ѧϰ����
    c2 = 1.4955; %ѧϰ����
    max_iter = 100;
    v_limit = 30;
    vel = -v_limit+v_limit*2*rand(nvars,N);
    [gbest, pbest, Xpfset] = pso_init(@GetFitness, Station(:,1), measurement_stacked(:,1), Xpfset);
    %%%%%%%%%%%%%%%%%%%%%% ģ��ϵͳ���� %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ��ʼ��������ʾ
    [X,Y,Z] = defMap(mapRange);
    init_map(X,Y,Z,Xpfset,Station,real_source)
    
    stop = T;
    for t=2:T
        if norm(Station(1:2,t -1) - real_source(1:2,t -1)) < mapRange/15
            stop = t-1;
            break;
        end
        % ģ��ϵͳ״̬����һ��
        Station(1:2,t) = Station(1:2,t-1) + 0.2*(Xpf(1:2,t-1) - Station(1:2,t-1));
        real_source(:,t) = real_source(:,t-1);  % ����ʵ��״ֵ̬
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
    %%%%%%%%%%%%%%%%%%%%%% ���ݷ��� %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % X��RMSƫ��Ƚ�ͼ
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
    legend([p1,p2,p5],'��ʵ״̬','�۲�״̬','PF����');
    xlabel('x��λ��');
    ylabel('y��λ��');
    zlabel('z��λ��');
    axis([0, mapRange, 0, mapRange]);
    view(3);
    
    figure
    hold on;
    box on;
    p1=plot(1:stop,PFXrms,'-k.','lineWidth',2);
    p2=plot(1:stop,PFYrms,'-m^','lineWidth',2);
    p3=plot(1:stop,PFZrms,'-ro','lineWidth',2);
    legend([p1,p2,p3],'PF-Xƫ��','PF-Yƫ��','PF-Zƫ��');
    xlabel('time step');
    ylabel('RMSԤ��ƫ��');
    
    figure;
    hold on;
    box on;
    p3=plot(1:stop,Tpf(1:stop),'-ro','lineWidth',2);
    legend([p3],'PFʱ��');
    xlabel('time step');
    ylabel('����ʱ��/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % �ٻ�һ����ͬQ��R�õ��Ĳ�ͬ�Ľ��ͼ
    % �ٻ�һ��ƫ������ͼ
    
    norm(real_source(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    