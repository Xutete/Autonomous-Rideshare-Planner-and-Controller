%%
clc
clear

%%
stops_total=6;

x_pos_init=5*randperm(10,stops_total);
x_pos_init(6)=25;
y_pos_init=5*randperm(10,stops_total);
y_pos_init(6)=25;
theta_init_deg=90*randi(4,1,stops_total);
theta_init_rad=theta_init_deg*2*pi/360;
w_init=zeros(1,6);
v_init=zeros(1,6);

%obstacles
num_obstacles=randperm(3,1);
x_obstacles=5*randperm(10,num_obstacles);
y_obstacles=5*randperm(10,num_obstacles);
r_obstacle=5;
x_collision=x_obstacles;
y_collision=y_obstacles;
for(k=1:1:num_obstacles)
    x=x_obstacles(k);
    y=y_obstacles(k);
    x_collision=[x_collision x+5 x x-5 x];
    y_collision=[y_collision y y+5 y y-5]; 
end

s=size(x_collision);
obstacle_nodes=s(2);
for(i=1:1:stops_total)
    for(j=1:1:obstacle_nodes)
    if(x_collision(j)==x_pos_init(i) && y_collision(j)==y_pos_init(i))
        disp("obstacle in collision")
        x_collision(j)=5*randperm(10,1);
        y_collision(j)=5*randperm(10,1);
        j=j-1;
    end
    end
end

%number of people to pick up
n_init=randi(4,1,6);
n_init(1)=0;
n_init(6)=n_init(2)+n_init(3)+n_init(4)+n_init(5);
if n_init(6)>10
    rider_err=1;
    if(rider_err==1)
        n_init=randi(4,1,6);
        n_init(1)=0;
        n_init(6)=n_init(2)+n_init(3)+n_init(4)+n_init(5);
    else
        rider_err=0;
    end
else
    return;
end

v=sin(theta_init_rad);
u=cos(theta_init_rad);
D=[x_pos_init;y_pos_init; theta_init_rad;v_init;w_init;n_init];

color=["r" "b" "m" "c" "g" "k"];
figure(1);

%plot intial points with legend identifying which point is which
subplot(2,3,1);
for i=1:1:length(x_pos_init)
scatter(x_pos_init(i),y_pos_init(i),color(i),'filled');
hold on;
end
legend('start','stop1','stop2','stop3','stop4','goal')

%%
%generating the cost matrix

ID=(1:121);
n_x=[];
n_y=[];

for(i=1:1:11)
   for(j=1:1:11)
    n_x_new=(j-1)*5;
    n_x=[n_x n_x_new];
    n_y_new=(i-1)*5;
    n_y=[n_y n_y_new];

   end
end

nodes=[ID;n_x;n_y]';

ID_s=(1:220);
s_1=[];
s_2=[];
for(m=1:1:11)
    for(n=1:1:10)
    s_1_new=n+(11*(m-1));
    s_1=[s_1 s_1_new];
    s_2_new=n+1+(11*(m-1));
    s_2=[s_2 s_2_new];
    end
end

for(o=1:1:10)
    for(p=1:1:11)
    s_1_new=p+(11*(o-1));
    s_1=[s_1 s_1_new];
    s_2_new=p+(11*(o-1))+11;
    s_2=[s_2 s_2_new];
    end
end

obstacle_node_id = zeros(obstacle_nodes,1);
for(j=1:obstacle_nodes)
    n=size(nodes);
    for(i=1:n(1))
        if(nodes(i,2)==x_collision(j) && nodes(i,3)==y_collision(j));
            obstacle_node_id(j) = nodes(i,1);
        end
    end
end

obstacle_node_id=obstacle_node_id';

segments=[ID_s;s_1;s_2]';

%Cost_matrix=genCost(D);
[Cost_matrix, paths,init_ID]=graphSearch(D,nodes,segments,obstacle_node_id);
[OptimalRoute,minCost]=OptimizeRoute(D(1:2,:)',Cost_matrix);

subplot(2,3,2);
plot(nodes(:,2), nodes(:,3),'k.');
hold on;

centers=[x_obstacles' y_obstacles'];
radii = r_obstacle*ones(num_obstacles,1);
viscircles(centers,radii,'Color','r');

% for(i=1:1:num_obstacles)
% h = circle(x_obstacles(i),y_obstacles(i),r_obstacle);
% hold on
% th = 0:pi/50:2*pi;
% xunit = r_obstacle * cos(th) + x_obstacles(i);
% yunit = r_obstacle * sin(th) + y_obstacles(i);
% h = plot(xunit, yunit);
% hold on;
% end

    for s = 1:220
         if (s <= 121) text(nodes(s,2),nodes(s,3),[' ' num2str(s)]); end
         plot(nodes(segments(s,2:3)',2),nodes(segments(s,2:3)',3),'k');
    end
    
    for(z=1:1:length(OptimalRoute)-1)
        path_storage(z).path_opt=paths(OptimalRoute(z),OptimalRoute(z+1)).optimal_path;

    end    
    
    %change init_ID - 1 2 3 4 5 6, now want this in order of optimal route
opt_ID=zeros(length(OptimalRoute),1);
for(p=1:1:length(OptimalRoute))
  opt_ID(p)=init_ID(1,OptimalRoute(p));
end
    
    
    for(z=1:1:length(OptimalRoute)-2)
        if(path_storage(z).path_opt(end-1)==path_storage(z+1).path_opt(2))
            bad_node=path_storage(z).path_opt(end-1);
            nodes_bad=nodes;
            nodes_bad(bad_node,2)=NaN;
            nodes_bad(bad_node,3)=NaN;
            [d, p] = dijkstra_final(nodes_bad, segments, opt_ID(z+1), opt_ID(z+2),obstacle_node_id);
            path_storage(z+1).path_new=p;
        else
            disp("no path change needed");
            path_storage(z+1).path_new=path_storage(z+1).path_opt;
           end
        path_storage(1).path_new=path_storage(1).path_opt
    end

for(z=1:1:length(OptimalRoute)-2)
        if(path_storage(z).path_new(end-1)==path_storage(z+1).path_opt(2))
            bad_node=path_storage(z).path_new(end-1);
            nodes_bad=nodes;
            nodes_bad(bad_node,2)=NaN;
            nodes_bad(bad_node,3)=NaN;
            [d, p] = dijkstra_final(nodes_bad, segments, opt_ID(z+1), opt_ID(z+2),obstacle_node_id);
            path_storage(z+1).path_new=p;
        else
            disp("no path change needed");
            path_storage(z+1).path_new=path_storage(z+1).path_opt;
           end
        path_storage(1).path_new=path_storage(1).path_opt
    end
    
    
for(k=1:1:length(OptimalRoute)-1)
    for t = 2:length(path_storage(k).path_new)
            %plot(nodes(paths(OptimalRoute(z),OptimalRoute(z+1)).optimal_path(t-1:t),2),nodes(paths(OptimalRoute(z),OptimalRoute(z+1)).optimal_path(t-1:t),3),'r-.','linewidth',2);
            plot(nodes(path_storage(k).path_new(t-1:t),2),nodes(path_storage(k).path_new(t-1:t),3),'r-.','linewidth',2);

    end
end
        
        hold on;

for i=1:1:length(x_pos_init)
scatter(x_pos_init(i),y_pos_init(i),color(i),'filled');
hold on;
end

%%
%segment seperator 
%single iteration
num_seg=zeros(length(OptimalRoute)-1);
for(k=1:1:length(OptimalRoute)-1)
    path_local = path_storage(k).path_new;
    horizontal = [];
    %1 means right, 2 means up, 3 means left, 4 means down
    for (g = 1:length(path_local)-1)
        if (path_local(g)-path_local(g+1) == 1)%moving left,3
            if (size((horizontal),1) == 0)%no turnings
                horizontal = [horizontal; path_local(g) 3];
            else
                if (~(horizontal(end) == 3))%previous one is not left
                    horizontal = [horizontal; path_local(g) 3];
                end
            end
        elseif (path_local(g)-path_local(g+1) == -1)%moving right,1
            if (size((horizontal),1) == 0)%no turnings
                horizontal = [horizontal; path_local(g) 1];
            else
                if (~(horizontal(end) == 1))%previous one is not left
                    horizontal = [horizontal; path_local(g) 1];
                end
            end
        elseif (path_local(g)-path_local(g+1) == -11)%moving up,2
            if (size((horizontal),1) == 0)%no turnings
                horizontal = [horizontal; path_local(g) 2];
            else
                if (~(horizontal(end) == 2))%previous one is not left
                    horizontal = [horizontal; path_local(g) 2];
                end
            end
        elseif (path_local(g)-path_local(g+1) == 11)%moving down 
            if (size((horizontal),1) == 0)%no turnings
                horizontal = [horizontal; path_local(g) 4];
            else
                if (~(horizontal(end) == 4))%previous one is not left
                    horizontal = [horizontal; path_local(g) 4];
                end
            end
        end
    end
    turn_info(k).turn_node= horizontal(2:end,1);
    turn_info(k).th_p= horizontal(1:end-1,2);
    turn_info(k).th_n= horizontal(2:end,2);
    turn_info(k).num_seg=length(horizontal(2:end,1))+1;
end

%trajectory generation

u=1;
v=1;
for(u=1:1:5)
    for(v=1:1:length(turn_info(u).th_p))
        if(turn_info(u).th_p(v)==1)
            turn_info(u).angle_p(v)=0;
        elseif(turn_info(u).th_p(v)==2)
            turn_info(u).angle_p(v)=(pi/2);
        elseif(turn_info(u).th_p(v)==3)
            turn_info(u).angle_p(v)=pi;
        else
            turn_info(u).angle_p(v)=(3*pi/2);
        end
    end
end

u=1;
v=1;
for(u=1:1:5)
    for(v=1:1:length(turn_info(u).th_n))
        if(turn_info(u).th_n(v)==1)
            turn_info(u).angle_n(v)=0;
        elseif(turn_info(u).th_n(v)==2)
               turn_info(u).angle_n(v)=(pi/2);
        elseif(turn_info(u).th_n(v)==3)
               turn_info(u).angle_n(v)=pi;
        elseif(turn_info(u).th_n(v)==4)
               turn_info(u).angle_n(v)=(3*pi/2);
        end
    end
end

total_segs=0;
counter=0;
for(i=1:1:length(OptimalRoute)-1) %number of segments/pieces in the route
    new_segs=turn_info(i).num_seg;
    for(j=1:1:new_segs) %number of segments in each piece of the route
             
        if(j==1&&i==1) %if first segment ever then make it the first condition possible
            counter=counter+1;
            D(3,1)=turn_info(1).angle_p(1);
            start=D(:,OptimalRoute(i));
            if(start(3)==3*pi/2)
                start(3)=-pi/2;
            end
        else %otherwise make the start condition be the end condition of the last segment
            counter=counter+1;
            start=acado_info(counter-1).finish_cond;
            
        end
        %final conditions - still looking at all the segments inside the
        %segments
        if(j==new_segs && j~=1 && i~=5) %if we are at the last segment in a section and there is not only 1 segment in the section and we are not in the last section of the route
            if(turn_info(i).th_n(j-1)==3 && turn_info(i+1).th_p(1)==4)
                D(3,OptimalRoute(i+1))=-3*pi/4;
            elseif(turn_info(i).th_n(j-1)==4 && turn_info(i+1).th_p(1)==3)
                D(3,OptimalRoute(i+1))=-3*pi/4;
            elseif(turn_info(i).th_n(j-1)==4 && turn_info(i+1).th_p(1)==1)
                D(3,OptimalRoute(i+1))=-pi/4;
            elseif(turn_info(i).th_n(j-1)==1 && turn_info(i+1).th_p(1)==4)
                D(3,OptimalRoute(i+1))=-pi/4;
            elseif(turn_info(i).th_n(j-1)==4 && turn_info(i+1).th_p(1)==4)
                D(3,OptimalRoute(i+1))=-pi/2;
            else
                D(3,OptimalRoute(i+1))=(turn_info(i).angle_n(j-1)+turn_info(i+1).angle_p(1))/2;
            end
            finish=D(:,OptimalRoute(i+1));
        elseif(j==new_segs && j==1 && i~=5) %last seg of the section and only 1 seg in the section but not the last section
            D(3,OptimalRoute(i+1))=(turn_info(i).angle_n(j)+turn_info(i+1).angle_p(1))/2;
            finish=D(:,OptimalRoute(i+1));
        elseif(j==new_segs && j~=1 && i==5) %last seg of the section not only 1 seg, also the last section of the path
                D(3,OptimalRoute(i+1))=turn_info(i).angle_n(j-1);
                finish=D(:,OptimalRoute(i+1));
                if(finish(3)==3*pi/2)
                    finish(3)=-pi/2;
                end
        elseif(j==new_segs && j==1 && i==5)% last seg of the section, only 1 seg, also the last section of the path
                D(3,OptimalRoute(i+1))=turn_info(i).angle_n(j);
                finish=D(:,OptimalRoute(i+1));
                if(finish(3)==3*pi/2)
                    finish(3)=-pi/2;
                end
        else
            x_f=nodes(turn_info(i).turn_node(j),2);
            y_f=nodes(turn_info(i).turn_node(j),3);
            if(turn_info(i).th_p(j)==3 && turn_info(i).th_n(j)==4)
                th_f=-3*pi/4;
            elseif(turn_info(i).th_p(j)==4 && turn_info(i).th_n(j)==3)
                th_f=-3*pi/4;
            elseif(turn_info(i).th_p(j)==4 && turn_info(i).th_n(j)==1)
                th_f=-pi/4;
            elseif(turn_info(i).th_p(j)==1 && turn_info(i).th_n(j)==4)
                th_f=-pi/4;
            elseif(turn_info(i).th_p(j)==4 && turn_info(i).th_n(j)==4)
                th_f=-pi/2;
            else
                th_f=(turn_info(i).angle_p(j)+turn_info(i).angle_n(j))/2;
            end
            v_f=0;
            w_f=0;
            finish=[x_f y_f th_f v_f w_f 0]';
        end
    acado_info(counter).path_num=i;
    acado_info(counter).seg_num=j;
    acado_info(counter).start_cond=start;
    acado_info(counter).finish_cond=finish;
    end
    total_segs=total_segs+new_segs;
end

%need to check for one of conditions that want us to go one direction or
%the other
k=1;
for(k=1:counter)
    dist = abs(acado_info(k).start_cond(3) - acado_info(k).finish_cond(3));
    acado_info(k).dist=dist;
    if(dist>pi)
        acado_info(k).change="yes";
        if(acado_info(k).start_cond(3)>0)
            acado_info(k).finish_cond(3)=acado_info(k).start_cond(3)+(2*pi-dist);
        else
            acado_info(k).finish_cond(3)=acado_info(k).start_cond(3)-(2*pi-dist);
        end
    end
end
%trajectory stitching

traj_x=[];
traj_y=[];
u1=[];
u2=[];
th=[];
v=[];
w=[];
t=[];
t_u=[];
err=[];

for(m=1:1:total_segs)
    traj_x=[traj_x []];
    traj_y=[traj_y []];
    th=[th []];
    v=[v []];
    w=[w []];
    t=[t []];
    u1=[u1 []];
    u2=[u2 []];
    t_u=[t_u []];
    err=[err []];
    start=acado_info(m).start_cond;
    goal=acado_info(m).finish_cond;
out(:,:,m)=acado_final(start,goal);
traj_x(:,m)=out(:,:,m).STATES(:,2);
traj_y(:,m)=out(:,:,m).STATES(:,3);
th(:,m)=out(:,:,m).STATES(:,4);
v(:,m)=out(:,:,m).STATES(:,5);
w(:,m)=out(:,:,m).STATES(:,6);
if(m==1)
    t(:,m)=out(:,:,m).STATES(:,1);
    t_u(:,m)=out(:,:,m).CONTROLS(:,1);
else    
    t(:,m)=t(end,m-1)+out(:,:,m).STATES(:,1);
    t_u(:,m)=t_u(end,m-1)+out(:,:,m).CONTROLS(:,1);
end
u1(:,m)=out(:,:,m).CONTROLS(:,2);
u2(:,m)=out(:,:,m).CONTROLS(:,3);
err(:,m)=sqrt((out(:,:,m).STATES(:,2)-finish(1)).^2+(out(:,:,m).STATES(:,3)-finish(2)).^2);
% if(start(1)-finish(1)==0)
%     err(:,m)=sqrt((out(:,:,m).STATES(:,2)-finish(1)).^2);
% else
%     err(:,m)=sqrt((out(:,:,m).STATES(:,3)-finish(2)).^2);
% end
end
%%
%resizing all data
s=size(t);
new_size=s(1)*s(2);
traj_x = reshape(traj_x,new_size,1);
traj_y = reshape(traj_y,new_size,1);
th = reshape(th,new_size,1);
for(i=1:1:new_size)
    if(th(i)>2*pi)
        th(i);
        th(i)=th(i)-2*pi;
    elseif(th(i)<-2*pi)
        th(i);
        th(i)=th(i)+2*pi;
    end
    %check if it is still not normalized to 2*pi to -2*pi
    if(th(i)>2*pi)
        i=i-1;
    elseif(th(i)<-2*pi)
        i=i-1;
    end
end
v = reshape(v,new_size,1);
for(i=1:1:new_size)
    if(v(i)<0)
        v(i);
        v(i)=v(i)*-1;
    end
end
w = reshape(w,new_size,1);
t = reshape(t,new_size,1);
u1 = reshape(u1,new_size,1);
u2 = reshape(u2,new_size,1);
err = reshape(err,new_size,1);
%%
%plotting data from trajectory generation
subplot(2,3,3);
hold on;
plot(traj_x,traj_y,'r')
xlabel('px') 
ylabel('py') 
title('Trajectory')

subplot(2,3,4);
hold on;
%plot(traj_x_T,traj_y_T,'r')
plot(t,u1,'-r',t,u2,'-b')
xlabel('t') 
ylabel('u') 
title('Control Effort')
legend('u1','u2')

subplot(2,3,2);
hold on;
%plot(traj_x_T,traj_y_T,'r')
plot(traj_x,traj_y,'-b')

subplot(2,3,5);
hold on;
%plot(traj_x_T,traj_y_T,'r')
plot(t,err,'-r')
xlabel('t') 
ylabel('error') 
title('Error')

subplot(2,3,6);
hold on;
%plot(traj_x_T,traj_y_T,'r')
plot(t,v,'-r',t,w,'-b')
xlabel('t') 
ylabel('velocity') 
title('Velocity')
legend('v','w')

%%
figure(2);

[ts,xas,t_true,error_true]=car_traj_fl_traj(traj_x,traj_y,th,v,w,t,u1,u2);
hold on;
plot(traj_x, traj_y, '-r')
hold on;
% visualize
plot(xas(:,1), xas(:,2), '-b');
%scatter(traj_x,traj_y,50,v,'filled') ; % velcoity as a scatter plot 
%colorbar
% hold on;

%visualize
%plot(xas(:,1), xas(:,2), '-b');
hold on;
%scatter(xas(:,1), xas(:,2),50,xas(:,4),'filled') ; % velcoity as a scatter plot 
%colorbar
legend('desired', 'executed')
title('With control limit')

figure(3);
plot(t_true,error_true,'-b')
xlabel('t')
ylabel('error')
title('Distance to Goal')
length = size(xas(:,4));
hold on;

for i = 1:length(1)
    if abs(xas(i,4)) < 0.01 
        plot(ts(i),xas(i,4),'*g')
    end
end

figure(4);
plot(ts,xas(:,4))
xlabel('t')
ylabel('v')
title('Velocity vs Time')
