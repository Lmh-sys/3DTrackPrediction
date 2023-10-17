% ����˵����ekf,ukf,pf,�Ľ�pf�㷨�����˻�����Ԥ��Ƚϳ���

function source

    clc; clear; close all;
    % �򱾳����漰̫���������������������ÿ�ζ�����
    % error('����Ĳ���T��ο����е�ֵ���ã�Ȼ��ɾ�����д���') 
    n = 3;
    T = 50; % ģ�ⳤ��
    
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % ϵͳ��ʼ��
    N = 200;         % �����˲�������    
    X = zeros(n,T);  % ��ʵֵ
    Z = zeros(1,T);  % �۲�ֵ
    % ��ʵ״̬��ʼ��
    range=150;
    step = range/1500;
    Station=zeros(2,T);   % �۲�վ��λ��
    X(:,1)=[range*0.9, range*0.6, 2.94e8]'+sqrtm(Q)*randn(n,1); % ��ʽѰԴ������Դ�������������ڹ������䳡�����ɹ۲�������
    Z(:,1) = underlying_model(Station(1:2,1),X(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF�˲��㷨 %%%%%%%%%%%%
    Xpf=zeros(n,T);        % �˲�״̬
    Xpf(:,1)=X(:,1);       % PF�˲�Ԥ��
    Xpfset=ones(n,N);      % ���Ӽ��ϳ�ʼ��
    Tpf=zeros(1,T);        % ���ڼ�¼һ���������ڵ��㷨ʱ������

    %%%%%%%%%%%%%%%%%%%%%% ģ��ϵͳ���� %%%%%%%%%%%%%%%%%%%%%%%%%
    for j=1:N   % ���Ӽ���ʼ��
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
    Xmap_pf(:,1)=X(:,1);       % PF�˲�Ԥ��
    for t=2:T
        % ģ��ϵͳ״̬����һ��
        Station(1:2,t) = Station(1:2,t-1) + step*(Xmap_pf(1:2,t-1) - Station(1:2,t-1));
        X(:,t) = X(:,t-1) + sqrtm(Q) * randn(n,1);  % ����ʵ��״ֵ̬
        Z(:,t) = underlying_model(Station(1:2,t),X(:,t)) + sqrtm(R)*randn(1,1);
        tic
        [Xpf(:,t),Xpfset,Neffpf]=pff(Xpfset,Z(:,t),N,n,R,Q,Station(1:2,t));                             % �㶨
        Tpf(t)=toc;
        for idex= 1:3
            [p, pos] = hist(Xpfset(idex, t), bins,2);
            map = find(p == max(p));
            Xmap_pf(idex, t) = pos(map(1));
        end
        Xpf(:,t) = Xmap_pf(:,t);
        

        if Neffpf < 10
            var_x = 100*step/t; % ���ӻ�����̬�ֲ�����ʱ��λ�õķ���var_p = 50,100 
            var_y = 100*step/t;
            % var_I = 4.5e7*t; % ���ӻ�����̬�ֲ�����ʱ��ǿ�ȵķ���
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
    
    %%%%%%%%%%%%%%%%%%%%%% ���ݷ��� %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % X��RMSƫ��Ƚ�ͼ
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
    legend([p1,p2,p5],'��ʵ״̬','�۲�״̬','PF����');
    xlabel('x��λ��');
    ylabel('y��λ��');
    zlabel('z��λ��');
    axis([0, range, 0, range]);
    view(3);
    
    figure
    hold on;
    box on;
    p1=plot(1:T,PFXrms,'-k.','lineWidth',2);
    p2=plot(1:T,PFYrms,'-m^','lineWidth',2);
    p3=plot(1:T,PFZrms,'-ro','lineWidth',2);
    legend([p1,p2,p3],'PF-Xƫ��','PF-Yƫ��','PF-Zƫ��');
    xlabel('time step');
    ylabel('RMSԤ��ƫ��');
    
    figure;
    hold on;
    box on;
    p3=plot(1:T,Tpf,'-ro','lineWidth',2);
    legend([p3],'PFʱ��');
    xlabel('time step');
    ylabel('����ʱ��/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % �ٻ�һ����ͬQ��R�õ��Ĳ�ͬ�Ľ��ͼ
    % �ٻ�һ��ƫ������ͼ
    
    norm(X(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    