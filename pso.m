function [gbest, pbest, pos,vel] = pso(obj_fun, lb, ub, max_iter, pop_size, Wmax, c1, c2, xk, cur_z, cur_partices, vel, pbest, gbest, v_limit)
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    % rand('seed',3);
    % randn('seed',6);
    Wmin = 0.5;
    pos = cur_partices(1:2,:);
    % 开始迭代
    for i = 1:max_iter
        W_pso = Wmin + (Wmax - Wmin) * exp(-3 * (i / max_iter) .^ 2);
        for n = 1: pop_size
            % 更新速度和位置
            v_id = vel(:,n);%i粒子的速度=(v_x v_y)
            x_id = pos(:,n);%i粒子的位置=(x_x x_y)
            p_id = pbest(1:2,n);%i粒子的个体历史最优位置
            r1 = rand(1,1);%r1,r2是介于[0,1]之间的随机数
            r2 = rand(1,1);
            v_id = W_pso*v_id + c1*r1*(p_id-x_id) + c2*r2*(gbest(1:2)-x_id);
            v_id( v_id> v_limit ) = v_limit;
            v_id( v_id< -v_limit ) = -v_limit; 
            x_id = x_id + v_id;
            %2) 对于越界的粒子进行调整: 反射边界上=速度大小不变 方向取反
            if ( x_id(1)>ub && v_id(1)>lb ) || ( x_id(1)< lb  && v_id(1)<0 )%x+或x-方向越界
                x_id = x_id - v_id;%恢复之前的位置
                v_id(1) = -v_id(1);%x方向上速度取反
                x_id = x_id + v_id;%然后再运动
            end
            if ( x_id(2)> ub && v_id(2)>lb ) || ( x_id(2)< lb && v_id(2)<0 )%y+或y-方向越界
                x_id = x_id - v_id;%恢复之前的位置
                v_id(2) = -v_id(2);%y方向上速度取反
                x_id = x_id + v_id;%然后再运动
            end
            pos(:,n) = x_id;
            vel(:,n) = v_id;
            w_partices = feval(obj_fun, xk, [pos(:,n); cur_partices(3,n)], cur_z);
            w_pbest = feval(obj_fun, xk,pbest(:,n), cur_z);
            w_gbest = feval(obj_fun, xk, gbest, cur_z);
            %刷新个体历史最优
            if w_partices > w_pbest
                pbest(:,n) = [pos(:,n); cur_partices(3,n)];
            end
            %刷新群体历史最优
            if w_partices > w_gbest
                gbest = [pos(:,n); cur_partices(3,n)];
            end
        end
        figure(2);hold off;
        plot(pos(1,:), pos(2,:),'.','Color',robot_color(1),'markersize',4);hold on
        % plot(xk(1),xk(2),'o','MarkerFaceColor',robot_color(2),'markersize',6);hold on
        axis([0, ub, 0, ub]);
        if w_pbest >= w_gbest*0.9 %粒子群最优值符合某个阀值的时候，退出循环，认为粒子已经分布在真实状态附近。
            break;
        end
    end
end


