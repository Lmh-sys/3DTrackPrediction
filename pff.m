%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���������˲��㷨
% ���룺XisetΪ��ά����
% �����XoΪnx1����XosetΪnxN����
function [Xo,Xoset,Neff]=pff(Xiset,Z,N,n,R,Q,Station)
 
    tic
    
    % �м������ʼ��
    Zpre=ones(1,N);     % �۲�Ԥ��   
    Xsetpre=ones(n,N);  % ���Ӽ���Ԥ��
    w = ones(1,N);      % Ȩֵ��ʼ��
    Xo=zeros(n,1);
    
    % ��һ��������ÿһ�����Ӷ�����ֲ����� 
    for i=1:N
        Xsetpre(:,i) = Xiset(:,i) + sqrtm(Q)*randn(n,1);
    end
    
    % �ڶ�������������Ȩ��
    for i=1:N
        Zpre(:,i) = underlying_model(Station,Xsetpre(:,i)) + sqrtm(R)*randn(1,1);
        z1 = Z-Zpre(:,i);
        % w(i) = inv(sqrtm(R)) * exp(-0.5*inv(R)*((Z-Zpre(:,i))^(2))) + 1e-99; 
        % poisspdf(floor(Z),Zpre(:,i));
        % normpdf(Z, ZZ,sqrt(Zpre(:,i)))
        w(i) = exp(-0.5 * ((Z - Zpre(:,i))./sqrt(Zpre(:,i))).^2) ./ (sqrt(2*pi) .* sqrt(Zpre(:,i)));
        
        % w(i) = inv(sqrt(2*pi*det(Zpre(:,i))))*exp(-0.5 * (z1)'*inv((Zpre(:,i)))*(z1))+ 1e-99;%Ȩֵ���㣬������ʵ���԰�inv(sqrt(2*pi*det(R)))ȥ��
    end
    
    % ��������Ȩ�ع�һ��
    w = (w + eps)./(sum(w) + eps);   
    
    % ����Ч������
    Neff = 1/sum(w.^2);
    
    % ���Ĳ����ز���
    if Neff < N*2/3
        outIndex = ResamplingRandom(1:N,w');
        % ���岽���������Ӽ��� 
        Xoset = Xsetpre(:,outIndex); 
        % ���������õ����μ���������˲�����ֵ
        % target = Xoset*w';
        target=[mean(Xoset(1,:)),mean(Xoset(2,:)),mean(Xoset(3,:))]';

    else
        Xoset = Xsetpre; 
        % target = Xoset*w';
        target = [mean(Xsetpre(1,:)),mean(Xsetpre(2,:)),mean(Xsetpre(3,:))]';
    end
    
    Xo(:,1)=target;
    
    