% main algorithm for Frenet path planning 
clc
clear 
close all
set(0,'DefaultLineLineWidth',1);
disp('Optimal Frenet Path Planning')




%% 2.set up the trajectory
% 2.1 set up the track
w_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
w_y=[0.0, -4.0, 1.0, 6.5, 9.0, 10.0, 6.0];

% 2.2 set up the obstacle
ob=[20.0, 10.0 ;
      30.0, 6.0 ;
      30.0, 5.0 ;
      35.0, 7.0 ;
      50.0, 12.0 ] ;
 % 2.3 create a reference track
  ds=0.1;    %discrete step size 
  GenerateTargetCourse = @(wx, wy) calcSplineCourse(wx, wy, ds);
  [RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath]=...
      GenerateTargetCourse(w_x, w_y);
 % [rx, ry, ryaw, rk, s, oSpline]=calcSplineCourse(x,y,ds);
  
 % Initial state
  s0_d=10.0 / 3.6;          % current speed [m/s]
  d0 = 2.0;                      % current lateral position [m]
  d0_d=0;                       % current lateral speed [m/s]
  d0_dd = 0;                    % current lateral acceleration [m/s]
  s0= 0;                          % current course position
  
  area=20;                      % animation area length[m]
  
  objFrenetPlanner = OptimalFrenetPlanner();
  show_animation=true;
  if show_animation
      figure
  end
      
  %start simulation 
  T=500;
  for t = 1:T 
        t;
        trajectory = objFrenetPlanner.FrenetOptimalPlanning (...
            referencePath, s0, s0_d, d0, d0_d, d0_dd, ob);
        
        %store the updated state of the planned trajectorut as initial
        %state of next iteration for the new trajectory
        s0 = trajectory.s(2);
        s0_d= trajectory.ds(2);
        d0 = trajectory.d(2);
        d0_d = trajectory.dd(2);
        d0_dd=trajectory.ddd(2);

  if(show_animation)
      cla;
      plot(RefX,RefY);
      hold on
      axis equal
      plot(ob(:,1),ob(:,2),'xk')
      plot(trajectory.x(1:end),trajectory.y(1:end), '-ob');
      plot(trajectory.x(1), trajectory.y(1), 'vc');
      grid on 
      drawnow
  end
  end