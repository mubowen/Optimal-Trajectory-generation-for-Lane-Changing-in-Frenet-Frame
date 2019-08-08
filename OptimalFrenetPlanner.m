classdef OptimalFrenetPlanner
    %define the frenet plannar algorithm
    properties
        % 1.set up parameters
MAX_SPEED=50.0/3.6;         % maximum speed [m/s]
MAX_ACCEL=2.0;                  % maximum accleration [m/ss]
MAX_CURVATURE=1.0;          % maximum curvature [1/m]
MAX_ROAD_WIDTH=7.0;       % maximum road width [m]
MIN_ROAD_WIDTH= -1.0      % minimum road width [m]
D_ROAD_W=1.0;                  % road width sampling length[m]
DT=0.2;                               % time tick [s]
MAXT=5.0;                           % max prediction time [s]
MINT=4.0;                            % min prediction time[s]
TARGET_SPEED=30.0/3.6;    % target speed [s]
D_T_S=5/3.6;                      % target speed sampling length [m/s]
N_S_SAMPLE=1;                   % sampling number of target speed
ROBOT_RADIUS=2.0;             % robot radius [m]

% cost weight
KJ=0.1;                                  % Jerk     
KT=0.1;                                 % time
KD=1.0;                                 % Distance from reference path
KV=1.0;                                 % Target speed
KLAT=1.0;                              % Lateral
KLON=1.0;                             % Longitudinal
        
numberObjects
    end
     
    methods ( Access = public)
     % \brief    Calculate trajectories in the frenet space
        % \details  Use the given start dynamics and generate trajectory variations
        %           by sampling 
        %           - The lateral position -max_road_width:delta_road_width:max_road_width 
        %           s0:     Longitudinal start position
        %           ds0:    Initial longitudinal velocity
        %           d0:     Lateral start position (offset from reference path)
        %           dd0:    Initial lateral velocity
        %           ddd0:    Initial lateral acceleration    
        function frenetTrajectories = CalcFrenetTrajectories(obj, s0, ds0,d0,dd0,ddd0)
            % generate path for each offset goal
            % Lateral sampling space
            sizeLatSampleSpace = length(obj.MIN_ROAD_WIDTH:obj.D_ROAD_W:obj.MAX_ROAD_WIDTH);
            sizeTimeSpace=length(obj.MINT:obj.DT:obj.MAXT);
            sizeLonSampleSpace=length((obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE):obj.D_T_S: ...
            (obj.TARGET_SPEED + obj.D_T_S * obj.N_S_SAMPLE));
            numberTrajectories=sizeLatSampleSpace*sizeTimeSpace*sizeLonSampleSpace;
            frenetTrajectories=cell(1, numberTrajectories);
            iTraj=1;
            for di=obj.MIN_ROAD_WIDTH: obj.D_ROAD_W:obj.MAX_ROAD_WIDTH
                %Lateral motion palnning
                for Ti=obj.MINT:obj.DT:obj.MAXT
                    
                    % Generate quintic polynomial for lateral plan using dynamics
                    % d0; start position offset
                    % dd0: start lateral velocity
                    % ddd0: start lateral acceleration
                    % di: Varoated lateral target lateral position
                    % ddT: lateral target velocity
                    % dddT: Lateral target acceleration
                    ddT=0;
                    dddT=0;
                    latPoly5 = QuinticPoly(d0, dd0, ddd0, di, ddT, dddT, Ti);
                    
                    %create a frenet trajetory consisting of 
                    % s (longitudinal) and d (lateral) dynamics
                    % and initialize the lateral (d) part
                    ft=FrenetTrajectory();
                    ft.t=0.0:obj.DT:Ti;
                    ft.d=latPoly5.X(ft.t);
                    ft.dd=latPoly5.dX(ft.t);
                    ft.ddd=latPoly5.ddX(ft.t);
                    ft.dddd=latPoly5.dddX(ft.t);
                    
                    % Longitudinal motion planning (velocity keeping)
                    for  tv = (obj.TARGET_SPEED - obj.D_T_S * obj.N_S_SAMPLE): obj.D_T_S: (obj.TARGET_SPEED...
                            + obj.D_T_S * obj.N_S_SAMPLE)
                          targetft=ft;
                          lonPoly4=QuarticPoly(s0, ds0, 0.0, tv, 0.0, Ti);
                          
                          targetft.s=lonPoly4.X(ft.t);
                          targetft.ds=lonPoly4.dX(ft.t);
                          targetft.dds=lonPoly4.ddX(ft.t);
                          targetft.ddds=lonPoly4.dddX(ft.t);
                          
                          % Square of lateral jerk
                          Jd = sum(targetft.ddds.^2);
                          % Square of longitudinal jerk
                          Js = sum(targetft.ddds.^2);
                          
                          % Square of diff from target speed
                          dv = (obj.TARGET_SPEED - targetft.ds(end)).^2;
                          
                          targetft.Jd = obj.KJ * Jd + obj.KT * Ti + obj.KD * targetft.d(end)^2 ;
                          targetft.Js = obj.KJ * Js + obj.KT * Ti + obj.KV * dv;
                          targetft.J=obj.KLAT * targetft.Jd + obj.KLON * targetft.Js;
                          
                           frenetTrajectories{iTraj} = targetft;
                           iTraj = iTraj + 1;
                    end
                end
            end
        end
        
        function frenetTrajectories = CalcGlobalTrajectories(obj, frenetTrajectories, referencePath)
            for iTarj = 1: length(frenetTrajectories)
                 ft = frenetTrajectories{iTarj};
                 % calc global positions
                 for i = 1:(length(ft.s))
                     [ix, iy] = referencePath.calc_position(ft.s(i));
                     if isnan(ix)
                         break
                     end
                     iyaw = referencePath.calc_yaw(ft.s(i));
                     di = ft.d(i);
                     fx = ix + di * cos(iyaw + pi /2.0);
                     fy = iy + di * sin(iyaw + pi / 2.0);
                     ft.x(end+1)=fx;
                     ft.y(end+1)=fy;
                     
                 end
                 plot(ft.x, ft.y, 'color', [1, 1, 1]*0.5)
                 drawnow;
                 
                 % calc theta and dL (running length)
                 for i = 1: (length(ft.x) - 1) 
                     dx = ft.x(i+1) - ft.x(i);
                     dy = ft.y(i+1) - ft.y(i);
                     ft.theta(end+1) = atan2(dy, dx);
                     ft.dL(end+1) = sqrt(dx^2 + dy^2);
                 end
                 
                 ft.theta(end+1) = ft.theta(end);
                 ft.dL(end+1) = ft.dL(end);
                 
                 % calc curvature
                 for i = 1: (length(ft.theta) - 1)
                     ft.kappa(end+1) = (ft.theta(i+1) - ft.theta(i)) / ft.dL(i) ;
                 end
                 ft.kappa(end+1) = ft.kappa(end);
                 
                 frenetTrajectories{iTarj} = ft;
                 
            end
        end
                 
        
        function collision = CheckCollision(obj, ft, objects)
            
                for i = 1:obj.numberObjects
                    ox = objects(i, 1);
                    oy = objects(i, 2);
                    d = zeros(length(ft.x), 1);
                    for idxPoint = 1:length(ft.x)
                        ix = ft.x(idxPoint);
                        iy = ft.y(idxPoint);
                        d(idxPoint) = ((ix - ox)^2 + (iy - oy)^2);
                    end
                    collision = any(d <= obj.ROBOT_RADIUS^2);
                    if collision
                        plot(ft.x, ft.y, 'rx')
                        plot(ox, oy, 'yo');
                        drawnow;
                        return;
                    end
                end
                collision = 0;
        end
        
        function okTrajectories = CheckTrajectories(obj, frenetTrajectories, objects)
            okTrajectories = {};
            for i = 1 : (length( frenetTrajectories))
                ft = frenetTrajectories{i};
                if any(ft.ds > obj.MAX_SPEED)  % Max speed check
                    continue
                elseif any(abs(ft.dds) > obj.MAX_ACCEL)     % Max accleration check
                    continue
                elseif any(abs(ft.kappa) > obj.MAX_CURVATURE)   % Max curvature check
                    continue
                end
                okTrajectories{end+1} = ft;
                plot(ft.x, ft.y, 'g');
                drawnow;
            end
        end
                
        function bestpath = FrenetOptimalPlanning(obj, referencePath, s0, ds0, d0, dd0, ddd0, objects)
            % Initialization
            obj.numberObjects = size(objects, 1);
            
            frenetTrajectories = obj.CalcFrenetTrajectories(s0, ds0, d0, dd0, ddd0);
            frenetTrajectories = obj.CalcGlobalTrajectories(frenetTrajectories, referencePath);
            frenetTrajectories = obj.CheckTrajectories(frenetTrajectories, objects);
            
            % Find minimum cost trajectory
            mincost = inf;
            bestpath = NaN;
            for iTraj = 1: (length(frenetTrajectories))
                ft = frenetTrajectories{iTraj};
                if (mincost >= ft.J)
                    mincost = ft.J;
                    bestpath = ft;
                end
            end
            
            collision = obj.CheckCollision(bestpath, objects);
        end
        
        function referencePath = CalcReferencePath(x, y, ds)
            referencePath = CalcSplineCourse(x, y, ds);
        end
        
    end
end
    
    
    
    
    
    
    
    
    
    
    
    
    