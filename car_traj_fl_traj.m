function [ts,xas,t_true,error_true] = car_traj_fl_traj(traj_x,traj_y,th,v,w,t,u1,u2)
% EN.530.678: HW#4 sample
% 1) compute a reference path using a polynomial in flat output space
% 2) track the path using feedback linearization
%
% M. Kobilarov, Spring 2014

%%%%%%%%% REFERENCE TRAJECTORY %%%%%%%%%%%%%

% plot(traj_x, traj_y, '-r')
% hold on


% %%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.x = traj_x;
S.y = traj_y;
S.th = th;
S.v = v;
S.w = w;
S.u1 = u1;
S.u2 = u2;
S.T = t;
index = size(u1);

du1 = (u1(2:index(1))-u1(1:index(1)-1))/0.1547;
du1 = [du1;du1(end)];
S.du1 = du1;
T = t(end);
error_true = [];
u_true = [];
t_true = [];

% gains
S.k = [5;5;8];


% augmented state with dynamic compensator, i.e xi=u1
x = [traj_x(1);traj_y(1);th(1);v(1);w(1)];
xa = [x+[.25;.25;.1;.1;.05]; S.u1(1)];

% simulate system
[ts, xas] = ode45(@uni_ode, [0 T], xa, [], S);

% % visualize
% plot(xas(:,1), xas(:,2), '-b');
% 
% legend('desired', 'executed')
% title('With control limit')
% figure
% plot(t_true,error_true,'-b')
% xlabel('t')
% ylabel('error')
% title('Distance to Goal')
% length = size(xas(:,4))
% hold on
% for i = 1:length(1)
%     if abs(xas(i,4)) < 0.01 
%         plot(ts(i),xas(i,4),'*g')
%     end
% end
% 
% figure
% plot(ts,xas(:,4))
% xlabel('t')
% ylabel('v')
% title('Velocity vs Time')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function A = poly5_coeff(y0, dy0, ddy0, yf, dyf, ddyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, ddy0, yf, dyf, ddyf]
L = [poly5(0), dpoly5(0), d2poly5(0),poly5(T), dpoly5(T), d2poly5(T)];
A = Y*inv(L);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = uni_h(x)
% output function

y = x(1:2);
end

function f = poly5(t)
f = [t.^5; t.^4; t.^3; t.^2; t; ones(size(t))];
end

function f = dpoly5(t)
f = [5*t.^4; 4*t.^3; 3*t.^2; 2*t; ones(size(t)); zeros(size(t))];
end

function f = d2poly5(t)
f = [20*t.^3; 12*t.^2; 6*t; 2*ones(size(t)); zeros(size(t)); zeros(size(t))];
end

function f = d3poly5(t)
f = [60*t.^2; 24*t; 6*ones(size(t)); zeros(size(t)); zeros(size(t)); zeros(size(t))];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ua = uni_ctrl(t, xa, S)
% tracking control law

% get desired outputs:
[M,i] = min(abs(S.T-t));
x_t = S.x(i);
y_t = S.y(i);
th_t = S.th(i);
v_t = S.v(i);
if abs(v_t) < 1e-4
    v_t = 1e-4;
end
w_t = S.w(i);
u_t = S.u1(i);
u2_t = S.u2(i);
du1_t = S.du1(i);
yd = [x_t;y_t];
dyd = [cos(th_t); sin(th_t)]*v_t;
d2yd = [u_t*cos(th_t)-v_t^2*sin(th_t)*tan(w_t);u_t*sin(th_t)+v_t^2*cos(th_t)*tan(w_t)];
d3yd = d3d(S.th(i),S.v(i),S.w(i),u_t,u2_t,du1_t);


% get current output
y = uni_h(xa);

% compensator, i.e.  xi=u1
xi = xa(end);
c3 = cos(xa(3));
s3 = sin(xa(3));
t5 = tan(xa(5));
s5 = sec(xa(5));
V = xa(4);
if abs(V)<0.1
    V = 0.1;
end
% current velocity
dy = [c3; s3]*V;
d2y = [xi*c3-V^2*s3*t5;xi*s3+V^2*c3*t5];

% error state
z1 = y - yd;
z2 = dy - dyd;
z3 = d2y - d2yd;
error_true = [error_true norm(z1)];

% virtual inputs
v = d3yd - S.k(1)*z1 -S.k(2)*z2 - S.k(3)*z3;

%offset
offset =[-2*s3*t5*xi*V-V^3*c3*t5^2-V*s3*t5*xi;
    2*c3*t5*xi*V+V*c3*t5*xi-s3*t5^2*V^3];

% augmented inputs ua=(dxi, u2)
if abs(det([c3 -s5^2*s3*V^2;s3 c3*s5^2*V^2]))<1e-3
    ua = inv([c3 -s5^2*s3*V^2;s3 c3*s5^2*V^2]+[1 1;1 1]*1e-3) * (v-offset);
else
    ua = inv([c3 -s5^2*s3*V^2;s3 c3*s5^2*V^2]) * (v-offset);
end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dxa = uni_ode(t, xa, S)

% unicycle ODE
ua = uni_ctrl(t, xa, S);
u_true = [u_true ua];
t_true = [t_true t];

xi = xa(end);
dxi = ua(1);
u2 = ua(2);
if xi > 5
xi = 5;
elseif xi < -5
xi = -5; 
end
if u2 > pi/6
u2 = pi/6;
elseif u2 < -pi/6
u2 = -pi/6; 
end

dxa = [cos(xa(3))*xa(4);
       sin(xa(3))*xa(4);
       tan(xa(5))*xa(4);
       xi;
       u2;
       dxi];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d3yd = d3d(th,v,w,u1,u2,du1)
d3yd = [-2*sin(th)*tan(w)*u1*v-v^3*cos(th)*tan(w)^2-v*sin(th)*tan(w)*u1+cos(th)*du1-sec(w)^2*sin(th)*v*u2;
        2*cos(th)*tan(w)*u1*v+v*cos(th)*tan(w)*u1-sin(th)*tan(w)^2*v^3+sin(th)*du1+sec(w)^2*cos(th)*v*u2];
end
end