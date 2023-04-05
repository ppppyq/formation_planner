leader_global_path_x = x_orgin.Data;
leader_global_path_y = y.Data;
leader_global_yaw = yaw.Data;
global_path_x = leader_global_path_x;
global_path_y = leader_global_path_y;
global_yaw = leader_global_yaw;
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


global_kappa = dheading_final./ds_final;
leader_global_kappa = global_kappa;