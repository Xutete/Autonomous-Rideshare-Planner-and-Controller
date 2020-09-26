function f = car_traj_fl()
% EN.530.678: HW#4 sample
% 1) compute a reference path using a polynomial in flat output space
% 2) track the path using feedback linearization
%
% M. Kobilarov, Spring 2014


% boundary conditions in state space
x0 = [-5; -3; 0 ; 0.5; 0];
xf = [0; 0; 1; 0.5; 0];
T = 10;
error_true = [];
u_true = [];
t_true = [];

%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%

% norm of initial and final velocity along desired path
% it determines how much the curves will bend
% and can be freely chosen
S.u1_start = 1;
S.u1_end = 0;
S.u2 = 0;

% boundary conditions in flat output space 
y0 = uni_h(x0);
yf = uni_h(xf);
dy0 = x0(4)*[cos(x0(3)); sin(x0(3))]; % desired starting velocity
dyf = xf(4)*[cos(xf(3)); sin(xf(3))]; % desired end velocity
d2y0 = [S.u1_start*cos(x0(3))-x0(4)^2*sin(x0(3))*tan(x0(5));S.u1_start*sin(x0(3))+x0(4)^2*cos(x0(3))*tan(x0(5))];
d2yf = [S.u1_end*cos(xf(3))-xf(4)^2*sin(xf(3))*tan(xf(5));S.u1_end*sin(xf(3))+xf(4)^2*cos(xf(3))*tan(xf(5))];


% compute path coefficients
A = poly5_coeff(y0, dy0, d2y0, yf, dyf, d2yf, T)

% plot desired path
X = A*poly5([0:.01:T]);
plot(X(1,:), X(2,:), '-r')
hold on


% %%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.A = A;

% gains
S.k = [5;5;8];

% perturb initial condition
x = x0 + [.25;.25;.1;.1;.05]

% augmented state with dynamic compensator, i.e xi=u1
xa = [x; S.u1_start];

% simulate system
[ts, xas] = ode45(@uni_ode, [0 T], xa, [], S);

% visualize
plot(xas(:,1), xas(:,2), '-b');

legend('desired', 'executed')
title('With control limit')

figure
plot(t_true,error_true,'-b')
xlabel('t')
ylabel('Norm(error)')
title('Distance to Goal')


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
yd = S.A*poly5(t);
dyd = S.A*dpoly5(t);
d2yd = S.A*d2poly5(t);
d3yd = S.A*d3poly5(t);

% get current output
y = uni_h(xa);

% compensator, i.e.  xi=u1
xi = xa(end);
c3 = cos(xa(3));
s3 = sin(xa(3));
t5 = tan(xa(5));
s5 = sec(xa(5));
V = xa(4);
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
ua = inv([c3 -s5^2*s3*V^2;s3 c3*s5^2*V^2]) * (v-offset);
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
   
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
