%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 基本粒子滤波算法
% 输入：Xiset为二维数组
% 输出：Xo为nx1矩阵，Xoset为nxN矩阵
function [Xo,Xoset,Neff]=pff(Xiset,Z,N,n,R,Q,Station)
 
    tic
    
    % 中间变量初始化
    Zpre=ones(1,N);     % 观测预测   
    Xsetpre=ones(n,N);  % 粒子集合预测
    w = ones(1,N);      % 权值初始化
    Xo=zeros(n,1);
    
    % 第一步，根据每一个粒子对先验分布采样 
    for i=1:N
        Xsetpre(:,i) = Xiset(:,i) + sqrtm(Q)*randn(n,1);
    end
    
    % 第二步，计算粒子权重
    for i=1:N
        Zpre(:,i) = underlying_model(Station,Xsetpre(:,i)) + sqrtm(R)*randn(1,1);
        z1 = Z-Zpre(:,i);
        % w(i) = inv(sqrtm(R)) * exp(-0.5*inv(R)*((Z-Zpre(:,i))^(2))) + 1e-99; 
        % poisspdf(floor(Z),Zpre(:,i));
        % normpdf(Z, ZZ,sqrt(Zpre(:,i)))
        w(i) = exp(-0.5 * ((Z - Zpre(:,i))./sqrt(Zpre(:,i))).^2) ./ (sqrt(2*pi) .* sqrt(Zpre(:,i)));
        
        % w(i) = inv(sqrt(2*pi*det(Zpre(:,i))))*exp(-0.5 * (z1)'*inv((Zpre(:,i)))*(z1))+ 1e-99;%权值计算，这里其实可以把inv(sqrt(2*pi*det(R)))去掉
    end
    
    % 第三步，权重归一化
    w = (w + eps)./(sum(w) + eps);   
    
    % 求有效粒子数
    Neff = 1/sum(w.^2);
    
    % 第四步，重采样
    if Neff < N*2/3
        outIndex = ResamplingRandom(1:N,w');
        % 第五步，更新粒子集合 
        Xoset = Xsetpre(:,outIndex); 
        % 第六步，得到本次计算的粒子滤波估计值
        % target = Xoset*w';
        target=[mean(Xoset(1,:)),mean(Xoset(2,:)),mean(Xoset(3,:))]';

    else
        Xoset = Xsetpre; 
        % target = Xoset*w';
        target = [mean(Xsetpre(1,:)),mean(Xsetpre(2,:)),mean(Xsetpre(3,:))]';
    end
    
    Xo(:,1)=target;
    
    