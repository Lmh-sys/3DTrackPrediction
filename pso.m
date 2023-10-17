function [gbest, pbest, pos,vel] = pso(obj_fun, lb, ub, max_iter, pop_size, Wmax, c1, c2, xk, cur_z, cur_partices, vel, pbest, gbest, v_limit)
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F"]);
    % rand('seed',3);
    % randn('seed',6);
    Wmin = 0.5;
    pos = cur_partices(1:2,:);
    % ��ʼ����
    for i = 1:max_iter
        W_pso = Wmin + (Wmax - Wmin) * exp(-3 * (i / max_iter) .^ 2);
        for n = 1: pop_size
            % �����ٶȺ�λ��
            v_id = vel(:,n);%i���ӵ��ٶ�=(v_x v_y)
            x_id = pos(:,n);%i���ӵ�λ��=(x_x x_y)
            p_id = pbest(1:2,n);%i���ӵĸ�����ʷ����λ��
            r1 = rand(1,1);%r1,r2�ǽ���[0,1]֮��������
            r2 = rand(1,1);
            v_id = W_pso*v_id + c1*r1*(p_id-x_id) + c2*r2*(gbest(1:2)-x_id);
            v_id( v_id> v_limit ) = v_limit;
            v_id( v_id< -v_limit ) = -v_limit; 
            x_id = x_id + v_id;
            %2) ����Խ������ӽ��е���: ����߽���=�ٶȴ�С���� ����ȡ��
            if ( x_id(1)>ub && v_id(1)>lb ) || ( x_id(1)< lb  && v_id(1)<0 )%x+��x-����Խ��
                x_id = x_id - v_id;%�ָ�֮ǰ��λ��
                v_id(1) = -v_id(1);%x�������ٶ�ȡ��
                x_id = x_id + v_id;%Ȼ�����˶�
            end
            if ( x_id(2)> ub && v_id(2)>lb ) || ( x_id(2)< lb && v_id(2)<0 )%y+��y-����Խ��
                x_id = x_id - v_id;%�ָ�֮ǰ��λ��
                v_id(2) = -v_id(2);%y�������ٶ�ȡ��
                x_id = x_id + v_id;%Ȼ�����˶�
            end
            pos(:,n) = x_id;
            vel(:,n) = v_id;
            w_partices = feval(obj_fun, xk, [pos(:,n); cur_partices(3,n)], cur_z);
            w_pbest = feval(obj_fun, xk,pbest(:,n), cur_z);
            w_gbest = feval(obj_fun, xk, gbest, cur_z);
            %ˢ�¸�����ʷ����
            if w_partices > w_pbest
                pbest(:,n) = [pos(:,n); cur_partices(3,n)];
            end
            %ˢ��Ⱥ����ʷ����
            if w_partices > w_gbest
                gbest = [pos(:,n); cur_partices(3,n)];
            end
        end
        figure(2);hold off;
        plot(pos(1,:), pos(2,:),'.','Color',robot_color(1),'markersize',4);hold on
        % plot(xk(1),xk(2),'o','MarkerFaceColor',robot_color(2),'markersize',6);hold on
        axis([0, ub, 0, ub]);
        if w_pbest >= w_gbest*0.9 %����Ⱥ����ֵ����ĳ����ֵ��ʱ���˳�ѭ������Ϊ�����Ѿ��ֲ�����ʵ״̬������
            break;
        end
    end
end


