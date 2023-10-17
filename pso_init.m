function [gbest, pbest, pos] = pso_init(obj_fun, xk, cur_z, cur_partices)
    
    pos = cur_partices;
    pbest = cur_partices;
    pbestval = feval(obj_fun, xk, cur_partices, cur_z);
    [~, gbest_idx] = max(pbestval);
    gbest = pbest(:,gbest_idx);
end


