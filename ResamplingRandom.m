function [out_particle_set] = ResamplingRandom(inIndex, weight_set)
    [particle_set_row, particle_set_col] = size(inIndex);
    %�ز���
    out_particle_set = ones(particle_set_row,particle_set_col);
    %����wc
    wc = cumsum(weight_set);
    %ת���ӣ�������������������ĸ�����
    %��������Ҫ�ز���n�����ӣ�������Ҫ��֮ǰ��ͬ
    for j=1:particle_set_col
        a=unifrnd(eps,1);%���ȷֲ�
        for k=1:particle_set_col
            if (a<wc(k))
                out_particle_set(:,j)=k;
                break;%%%%һ��Ҫbreak�������ز������ӻᱻ���һ�����Ӹ��ǣ�������µĵ�ʮ��
            end
        end
    end
    %�ز������
end