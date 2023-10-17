function A = underlying_model(xk,source_vector)
    % u_i = 0.05;
    xk = floor(xk);
    source_vector = floor(source_vector);
    R = (source_vector(1,:) -xk(1)).^2 + (source_vector(2,:) -xk(2)).^2;
%     R = norm(xk(1:2)-source_vector(1:2)) + eps;
    A = ((source_vector(3,:)*3.14e-3*0.3) ./ (R) + 10) * 10;
    % A = source_vector(3,1)/(R^2) + 100;
    A(R < 0.1) = source_vector(3,R < 0.1);
%     if R < 0.1
%         A = source_vector(3,1);
%     end

end

function output = myFun(input)
    %% 原始模型
    clear
    clc
    x_range = [0, 150]; % 搜素区域的大小
    y_range = [0, 150];
    source_vector = [x_range(2)*0.9, y_range(2)*0.6, 2.94e8]'; % 正式寻源，放射源参数向量，用于构建辐射场，生成观测向量集
    [X,Y] = meshgrid(0:1:150);
    R = (X -source_vector(1)).^2 + (Y -source_vector(2)).^2;
    theory_value = ((source_vector(3,1)*3.14e-3*0.3) ./ (R) + 10) * 10;
    A = floor(random('Poisson',theory_value,size(theory_value)));
    % 生成x和y的网格点
    meshc(X,Y,A);
    [C,h] = contour(X,Y,A)
    clabel(C,h)
    
    
    %% 改扩散模型
    % 效果不错
    clear
    clc
    range = 900;
    source_vector = [range*0.9, range*0.6, 2.94e8]'; % 正式寻源，放射源参数向量，用于构建辐射场，生成观测向量集
    u_i = 0.05;% 第i中物质的衰减系数
    [X,Y] = meshgrid(0:5:range);
    R = (X -source_vector(1)).^2 + (Y -source_vector(2)).^2;
    theory_value = (source_vector(3,1) ./ (R));
    A = floor(random('Poisson',theory_value,size(theory_value)));
    A(isinf(A(:))) = 2.8e8;
    % 画出结果图
    figure;
    surf(X,Y,A);
    % colormap(jet);
    % % shading interp;
    % view([0,90]);
end