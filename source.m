% ����˵����ekf,ukf,pf,�Ľ�pf�㷨�����˻�����Ԥ��Ƚϳ���

function source

    clc; clear; close all;
    % �򱾳����漰̫���������������������ÿ�ζ�����
    % error('����Ĳ���T��ο����е�ֵ���ã�Ȼ��ɾ�����д���') 
    dim = 3;
    T = 50; % ģ�ⳤ��
    
    Q= diag([0.1,0.1,0.01]);
    R = 0.01;
    % ϵͳ��ʼ��
    N = 200;         % �����˲�������    
    X = zeros(dim,T);  % ��ʵֵ
    Z = zeros(1,T);  % �۲�ֵ
    % ��ʵ״̬��ʼ��
    range=150;
    step = range/1500;
    Station=zeros(2,T);   % �۲�վ��λ��
    X(:,1)=[range*0.9, range*0.6, 2.94e8]'+sqrtm(Q)*randn(dim,1); % ��ʽѰԴ������Դ�������������ڹ������䳡�����ɹ۲�������
    Z(:,1) = underlying_model(Station(1:2,1),X(:,1)) + sqrtm(R)*randn(1,1);
    strength_range = [3e7, 1e9];

    %%%%%%%%%%%%% PF�˲��㷨 %%%%%%%%%%%%
    Xpf=zeros(dim,T);        % �˲�״̬����
    Xpf(:,1)=X(:,1);       % PF�˲�Ԥ��
    Xpfset=ones(dim,N);      % ���Ӽ��ϳ�ʼ��
    Tpf=zeros(1,T);        % ���ڼ�¼һ���������ڵ��㷨ʱ������
    rand('seed',3);
    randn('seed',6);
    for j=1:N   % ���Ӽ���ʼ��
        Xpfset(1, j) = unifrnd(0,range);
        Xpfset(2, j) = unifrnd(0,range);
        % Xpfset(3, j) = 2.94e8;
        Xpfset(3, j) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    end
    %%%%%%%%%%%%% PSO ����Ⱥ %%%%%%%%%%%%
    nn = N; %����Ⱥ��ģ���������� %���ӵ�ά������Ѱ�Ų����ĸ���
    pso_dim = 6;
    Wmax = 0.9; % ����ϵ��
    Wmin = 0.5;
    iter= 1;
    ITERmax = 100; %��������
    c1 = 1.4955; %ѧϰ����
    c2 = 1.4955; %ѧϰ����
    pose_limit = range; % λ������
    alpha = 1;%Լ������=�����ٶȵ�Ȩ��
    v_limit = 30;%�ٶ�����

    X_pso = zeros(pso_dim,nn); %��ʼ������Ⱥ[��� λ��x λ��y ���I �ٶ�x �ٶ�y]
    Pbest = zeros(4,nn); %������ʷ���� ÿ��=[��� λ��x λ��y ���I] 
    Gbest = zeros(3,1); %Ⱥ����ʷ���� ÿ��=[λ��x λ��y ���I] (��������Ⱥ����һ��)
    pres_Pbest = zeros(1,nn);
    Gbest(1) = pose_limit*rand(1,1);% ȫ�����ų�ֵ
    Gbest(2) = pose_limit*rand(1,1);% ȫ�����ų�ֵ
    Gbest(3) = unifrnd(strength_range(1, 1), strength_range(1, 2));
    vel = -v_limit+v_limit*2*rand(2,N);
    %��������Ⱥ��ֵ(ÿ���������λ�ú��ٶ�)
    for i = 1: 1: nn
        %ÿ������
        X_pso(1,i) = i;%���
        X_pso(2,i) = Xpfset(1,i);
        X_pso(3,i) = Xpfset(2,i);
        X_pso(4,i) = Xpfset(3,i);
        X_pso(5,i) = vel(1,i);%vx=-v_limit~v_limit
        X_pso(6,i) = vel(2,i);%vy=-v_limit~v_limit
        %������ʷ����
        Pbest(1,i) = X_pso(1,i);%���
        Pbest(2,i) = X_pso(2,i);%x
        Pbest(3,i) = X_pso(3,i);%y
        Pbest(4,i) = X_pso(4,i);%y
        %Ⱥ����ʷ���ų�ֵ
        w_pbest = GetFitness(Station(1:2,1), Pbest(2:4,i), Z(:,1));
        pres_Pbest(i) = w_pbest;
        w_gbest = GetFitness(Station(1:2,1), Gbest, Z(:,1));
        if w_pbest > w_gbest
            Gbest = Pbest(2:4,i);
        end
    end
    % [gbest, gbestval] = psoff(@GetFitness, pso_dim, 0, range, 100, pop_size,0.9, c1, c2);
    %%%%%%%%%%%%%%%%%%%%%% ģ��ϵͳ���� %%%%%%%%%%%%%%%%%%%%%%%%%
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    % ��ʼ��������ʾ
    figure(1);hold off;
    plot(X_pso(2, :), X_pso(3, :),'.','Color',robot_color(1),'markersize',4);hold on
    plot(Station(1,1),Station(2,1),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
    plot(X(1,1),X(2,1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
    bins = 20;
    Xmap_pf = zeros(3, T);
    Xmap_pf(:,1)=X(:,1);       % PF�˲�Ԥ��
    for t=2:T
        if norm(Station(1:2,t -1) - X(1:2,t -1)) < range/15
            break;
        end
        % ģ��ϵͳ״̬����һ��
        Station(1:2,t) = Station(1:2,t-1) + 0.2*(Xpf(1:2,t-1) - Station(1:2,t-1));
        X(:,t) = X(:,t-1) + sqrtm(Q) * randn(dim,1);  % ����ʵ��״ֵ̬
        Z(:,t) = underlying_model(Station(1:2,t),X(:,t)) + sqrtm(R)*randn(1,1);
        % ����Ⱥ����
        while (iter < ITERmax)
            for i = 1: 1: nn%ÿ������
                v_id = X_pso(5:6,i);%i���ӵ��ٶ�=(v_x v_y)
                x_id = X_pso(2:3,i);%i���ӵ�λ��=(x_x x_y)
                p_id = Pbest(2:3,i);%i���ӵĸ�����ʷ����λ��
                r1 = rand(1,1);%r1,r2�ǽ���[0,1]֮��������
                r2 = rand(1,1);
                % W_pso=Wmax-(Wmax-Wmin)/ITERmax*iter; %��������
                W_pso = Wmin + (Wmax - Wmin) * exp(-3 * (iter / ITERmax) .^ 2);
                %  W_pso=(W_pso1+W_pso2)/2;

                v_id = W_pso*v_id + c1*r1*(p_id-x_id) + c2*r2*(Gbest(1:2)-x_id);
                v_id( v_id> v_limit ) = v_limit;
                v_id( v_id< -v_limit ) = -v_limit; 
                x_id = x_id + alpha*v_id;
                %2) ����Խ������ӽ��е���: ����߽���=�ٶȴ�С���� ����ȡ��
                if ( x_id(1)>range && v_id(1)>0 ) || ( x_id(1)< 0  && v_id(1)<0 )%x+��x-����Խ��
                    x_id = x_id - alpha*v_id;%�ָ�֮ǰ��λ��
                    v_id(1) = -v_id(1);%x�������ٶ�ȡ��
                    x_id = x_id + alpha*v_id;%Ȼ�����˶�
                end
                if ( x_id(2)> range && v_id(2)>0 ) || ( x_id(2)< 0 && v_id(2)<0 )%y+��y-����Խ��
                    x_id = x_id - alpha*v_id;%�ָ�֮ǰ��λ��
                    v_id(2) = -v_id(2);%y�������ٶ�ȡ��
                    x_id = x_id + alpha*v_id;%Ȼ�����˶�
                end
                %3) �����˶�
                %ˢ�����ӵ�λ�ú��ٶ�
                X_pso(2:3,i) = x_id;
                X_pso(5:6,i) = v_id;
                w_partices = GetFitness(Station(1:2,t), X_pso(2:4,i), Z(:,t));
                w_pbest = GetFitness(Station(1:2,t), Pbest(2:4,i), Z(:,t));
                pres_Pbest(i) = w_pbest;
                w_gbest = GetFitness(Station(1:2,t), Gbest, Z(:,t));
                %ˢ�¸�����ʷ����
                if w_partices > w_pbest
                    Pbest(2:4,i) = X_pso(2:4,i);
                end
                %ˢ��Ⱥ����ʷ����
                if w_partices > w_gbest
                    Gbest = X_pso(2:4,i);%tmp_pg ��Ӱ����һ�ֺ����������ж�
                end
            end
            % ���ӻ�
            figure(1);hold off;
            plot(X_pso(2, :), X_pso(3, :),'.','Color',robot_color(1),'markersize',4);hold on
            plot(Station(1,t),Station(2,t),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
            plot(X(1,1),X(2,1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
            plot(Xmap_pf(1,t-1),Xmap_pf(2,t-1),'o','MarkerFaceColor',robot_color(2),'markersize',4);hold on
            axis([0, range, 0, range]);
            if w_pbest >= w_gbest*0.9 %����Ⱥ����ֵ����ĳ����ֵ��ʱ���˳�ѭ������Ϊ�����Ѿ��ֲ�����ʵ״̬������
                iter = ITERmax;
            else
                iter = iter + 1;
            end
        end %����Ⱥѭ������
        iter = 1 ;
        tic
        % [Xpf(:,t),Xpfset,Neffpf]=pff(Xpfset,Z(:,t),N,dim,R,Q,Station(1:2,t));        
        [Xpf(:,t),X_pso(2:4,:),Neffpf]=pff(X_pso(2:4,:),Z(:,t),N,dim,R,Q,Station(1:2,t));                             % �㶨
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
        %     % var_x = 100*step/t; % ���ӻ�����̬�ֲ�����ʱ��λ�õķ���var_p = 50,100 
        %     % var_y = 100*step/t;
        %     % % var_I = 4.5e7*t; % ���ӻ�����̬�ֲ�����ʱ��ǿ�ȵķ���
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
    
    % figure;
    % hold on;
    % box on;
    % p3=plot(1:T,Tpf,'-ro','lineWidth',2);
    % legend([p3],'PFʱ��');
    % xlabel('time step');
    % ylabel('����ʱ��/s');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % �ٻ�һ����ͬQ��R�õ��Ĳ�ͬ�Ľ��ͼ
    % �ٻ�һ��ƫ������ͼ
    
    norm(X(1:2,end) - Station(1:2,end))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    