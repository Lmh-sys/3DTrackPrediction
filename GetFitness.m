
function  w= GetFitness(agent, pose, Z)
    % agent: ��ǰ�۲�վ 2ά
    % pose: ����λ�úͻ�� 200*3ά
    % Z: ��ǰ�۲�
    Zpre = underlying_model(agent,pose);
    % w = - abs(Z - Zpre);
    % w=exp(-1/2*(Z - Zpre).^2);
    % w = poisspdf(floor(Z),Zpre);
    w = exp( -((Z - Zpre)./sqrt(Zpre)).^2) ;
end