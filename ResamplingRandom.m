function [out_particle_set] = ResamplingRandom(inIndex, weight_set)
    [particle_set_row, particle_set_col] = size(inIndex);
    %重采样
    out_particle_set = ones(particle_set_row,particle_set_col);
    %生成wc
    wc = cumsum(weight_set);
    %转盘子，生成随机数，看落在哪个区间
    %首先我们要重采样n个粒子，粒子数要与之前相同
    for j=1:particle_set_col
        a=unifrnd(eps,1);%均匀分布
        for k=1:particle_set_col
            if (a<wc(k))
                out_particle_set(:,j)=k;
                break;%%%%一定要break，否则重采样粒子会被最后一个粒子覆盖，具体见新的第十讲
            end
        end
    end
    %重采样完毕
end