function [gbest, pbest, pos,vel] = psoff(obj_fun, lb, ub, max_iter, nvars, pop_size, Wmax, c1, c2, xk, cur_z, cur_partices, vel, pbest, gbest, v_limit)
    % obj_fun: 目标函数句柄
    % nvars: 优化变量的数量
    % lb: 变量下界向量
    % ub: 变量上界向量
    % max_iter: 最大迭代次数
    % pop_size: 群体大小
    % w: 惯性因子
    % c1: 认知因子
    % c2: 社会因子
    % obj_fun = GetFitness;
    % c1 = 1.4955; %学习因子
    % c2 = 1.4955; %学习因子
    % nvars = 2;
    % range = 150;
    % pop_size = 200;
    % xk = zeros(1,3);
    % cur_z = 108;
    % psoff(@obj_fun, nvars, 0, range, 100, 200, 0.9, c1, c2, xk, cur_z)
    % xk 观测站
    % cur_z 当前观测
    % 初始化粒子群
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    rand('seed',3);
    randn('seed',6);
    Wmin = 0.5;
    Wmin = 0.5;
    pos = cur_partices(1:2,:);
    
    % 开始迭代
    for i = 1:max_iter
        % 更新速度和位置
        r1 = rand(nvars, pop_size);
        r2 = rand(nvars, pop_size);
        w = Wmin + (Wmax - Wmin) * exp(-3 * (i / max_iter) .^ 2);
        vel = w .* vel + c1 .* r1 .* (pbest(1:2,:) - pos) + c2 .* r2 .* (repmat(gbest(1:2), 1, pop_size) - pos);
        vel( vel> v_limit ) = v_limit;
        vel( vel< -v_limit ) = -v_limit; 
        pos = pos + vel;

        % 边界处理
        pos(pos < lb) = lb;
        pos(pos > ub) = ub;

        % 计算适应值并更新个体最优解和全局最优解
        w_partices = feval(obj_fun, xk, [pos; cur_partices(3,:)], cur_z);
        w_pbest = feval(obj_fun, xk,pbest, cur_z);
        w_gbest = feval(obj_fun, xk, gbest, cur_z);
        topbest_update = w_partices > w_pbest;
        
        pbest(:,topbest_update) = [pos(:,topbest_update); cur_partices(3,topbest_update)];

        [cur_gbestval, cur_gbest_idx] = max(w_pbest);
        if cur_gbestval > w_gbest
            gbest = pbest(:, cur_gbest_idx);
        end
        figure(1);hold off;
        plot(pos(1,:), pos(2,:),'.','Color',robot_color(1),'markersize',4);hold on
        plot(xk(1),xk(2),'o','MarkerFaceColor',robot_color(4),'markersize',4);hold on
        axis([0, ub, 0, ub]);
        if w_gbest(end) >= w_gbest*0.9 %粒子群最优值符合某个阀值的时候，退出循环，认为粒子已经分布在真实状态附近。
            break;
        end
    end
end


