function [path_x_out,path_y_out,path_yaw_out,path_kappa_out]= path_projection(path_x,path_y,path_yaw,path_kappa,l)
    if(l == 0)
        path_x_out = path_x;
        path_y_out = path_y;
        path_yaw_out = path_yaw;
        path_kappa_out = path_kappa;
    else
        n = length(path_x);
        path_x_out = zeros(n,1);
        path_y_out = zeros(n,1);
        path_yaw_out = zeros(n,1);
        path_kappa_out = zeros(n,1);        
        index2s = calindex2s(path_x,path_y);
        s_set = index2s;
        l_set = ones(n,1)*l;
        for i = 1:n
            % 计算(s,l)在frenet坐标轴上的投影
            [proj_x,proj_y,proj_heading,proj_kappa] = CalcProjPoint(s_set(i),path_x,path_y,path_yaw,...
        path_kappa,index2s);
            nor = [-sin(proj_heading);cos(proj_heading)];
            point = [proj_x;proj_y] + l_set(i) * nor;
            path_x_out(i) = point(1);
            path_y_out(i) = point(2);
            path_yaw_out(i) = proj_heading;
            % 近似认为 kappa' == 0,frenet转cartesian公式，见第一章第三节评论区的链接
            global_path_x = path_x_out;
            global_path_y = path_y_out;
            global_yaw = path_yaw_out;
            dx = diff(global_path_x);
            dy = diff(global_path_y);
            
            
            dx_pre = [dx(1);dx];
            dx_after = [dx;dx(end)];
            dx_final = (dx_pre+dx_after)/2;
            
            
            dy_pre = [dy(1);dy];
            dy_after = [dy;dy(end)];
            dy_final = (dy_pre+dy_after)/2;
            
            
            ds_final = sqrt(dx_final.^2+dy_final.^2);
            
            
            dheading = diff(global_yaw);
            dheading_pre = [dheading(1);dheading];
            dheading_after = [dheading;dheading(end)];
            dheading_final = (dheading_pre+dheading_after)/2;
            
            
            path_kappa_out = dheading_final./ds_final;
             
        end
    end

end


function [proj_x,proj_y,proj_heading,proj_kappa] = CalcProjPoint(s,frenet_path_x,frenet_path_y,frenet_path_heading,...
    frenet_path_kappa,index2s)
% 该函数将计算在frenet坐标系下，点(s,l)在frenet坐标轴的投影的直角坐标(proj_x,proj_y,proj_heading,proj_kappa)'
% 先找匹配点的编号
    match_index = 1;
    while index2s(match_index) < s
        match_index = match_index + 1;
    end
    match_point = [frenet_path_x(match_index);frenet_path_y(match_index)];
    match_point_heading = frenet_path_heading(match_index);
    match_point_kappa = frenet_path_kappa(match_index);
    ds = s - index2s(match_index);
    match_tor = [cos(match_point_heading);sin(match_point_heading)];
    proj_point = match_point + ds * match_tor;
    proj_heading = match_point_heading + ds * match_point_kappa;
    proj_kappa = match_point_kappa;
    proj_x = proj_point(1);
    proj_y = proj_point(2);   
end

function index2s = calindex2s(path_x,path_y)
    n = length(path_x);
    index2s = zeros(n,1);
    for i = 2:n
        index2s(i) = sqrt((path_x(i) - path_x(i-1))^2 + (path_y(i) - path_y(i-1))^2) + index2s(i-1);
    end
end