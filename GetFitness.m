
function  w= GetFitness(agent, pose, Z)
    % agent: 当前观测站 2维
    % pose: 粒子位置和活度 200*3维
    % Z: 当前观测
    Zpre = underlying_model(agent,pose);
    % w = - abs(Z - Zpre);
    % w=exp(-1/2*(Z - Zpre).^2);
    % w = poisspdf(floor(Z),Zpre);
    w = exp( -((Z - Zpre)./sqrt(Zpre)).^2) ;
end