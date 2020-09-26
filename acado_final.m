function [out]=acado_final(start,finish)
%clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'hw6_car');
 
    t_f=10;
    num_steps=64;
    
    % (a) Define states and controls 
DifferentialState p_x;
DifferentialState p_y;
DifferentialState theta;
DifferentialState v;
DifferentialState delta;

Control ua;
Control udelta;
   
    %% (a) Differential Equation

f=acado.DifferentialEquation(0,t_f);

f.add(dot(p_x)==v*cos(theta));
f.add(dot(p_y)==v*sin(theta));
f.add(dot(theta)==v*tan(delta));
f.add(dot(v)==ua);
f.add(dot(delta)==udelta);

    
    %% (b) Optimal Control Problem
    ocp=acado.OCP(0.0, t_f, num_steps);
                      
    %(b) Minimize control effort
    u=[ua udelta v]; 
    S=[2 0 0;0 1 0;0 0 1];
    ocp.minimizeLSQ(S,u,[0;0;0]);
% 
%     u=[ua udelta]; 
%     S=[2 0 ;0 1 ];
%     ocp.minimizeLSQ(S,u,[0;0]);

    % (c) Path constraints
     ocp.subjectTo(0<=v<=5);
     ocp.subjectTo(-5<=ua<=5);
     ocp.subjectTo(-pi/4<=delta<=pi/4);
     ocp.subjectTo(-pi/6<=udelta<=pi/6);
     ocp.subjectTo(f);
     
    
    % (d) Initial Conditions
        x_pos_i=start(1);
        y_pos_i=start(2);
        theta_i=start(3);
        v_i=start(4);
        w_i=start(5);
   
   ocp.subjectTo('AT_START', p_x==x_pos_i);
   ocp.subjectTo('AT_START', p_y==y_pos_i);
   ocp.subjectTo('AT_START', v==v_i);
   ocp.subjectTo('AT_START', theta==theta_i);
   ocp.subjectTo('AT_START', delta==w_i);
    
    % (d) Final boundary conditions
    x_pos_f=finish(1);
    y_pos_f=finish(2);
    theta_f=finish(3);
    v_f=finish(4);
    w_f=finish(5);
   
   ocp.subjectTo('AT_END', p_x==x_pos_f);
   ocp.subjectTo('AT_END', p_y==y_pos_f);
   ocp.subjectTo('AT_END', v==v_f);
   ocp.subjectTo('AT_END', theta==theta_f);
   ocp.subjectTo('AT_END', delta==w_f);
    
    %% (e) Optimization Algorithm
    algo=acado.OptimizationAlgorithm(ocp);
    algo.set('KKT_TOLERANCE',1e-8);
    algo.set('DISCRETIZATION_TYPE','MULTIPLE_SHOOTING'); 
    algo.set('MAX_NUM_ITERATIONS',500);
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
  out = hw6_car_RUN();
end