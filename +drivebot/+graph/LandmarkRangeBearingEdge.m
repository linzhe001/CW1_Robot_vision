classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This class stores an edge which represents the factor for observing
    % the range and bearing of a landmark from the vehicle. Note that the
    % sensor is fixed to the platform.
    %
    % The measurement model is
    %
    %    z_(k+1)=h[x_(k+1)]+w_(k+1)
    %
    % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
    % The sensor is at (lx, ly).
    %
    %    dx = lx - x_(k+1); dy = ly - y_(k+1)
    %
    %    r(k+1) = sqrt(dx^2+dy^2)
    %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
    %
    % The error term
    %    e(x,z) = z(k+1) - h[x(k+1)]
    %
    % However, remember that angle wrapping is required, so you will need
    % to handle this appropriately in compute error.
    %
    % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
    
    methods(Access = public)
    
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge(landmark);
            %
            % Description:
            %   Creates an instance of the LandmarkRangeBearingEdge object.
            %   Note we feed in to the constructor the landmark position.
            %   This is to show there is another way to implement this
            %   functionality from the range bearing edge from activity 3.
            %
            % Inputs:
            %   landmark - (2x1 double vector)
            %       The (lx,ly) position of the landmark
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a ObjectGPSMeasurementEdge

            obj = obj@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of the landmark.
            % 使用车辆位姿和测量数据来估计地标初始位置
            pose = obj.edgeVertices{1}.x; % 车辆位姿 [x; y; theta]
            meas = obj.measurement;       % 测量数据 [range; bearing]
            r = meas(1);
            beta = meas(2);
            % 根据车辆位姿和测量更新地标位置估计
            lx = pose(1) + r * cos(pose(3) + beta);
            ly = pose(2) + r * sin(pose(3) + beta);
            obj.edgeVertices{2}.setEstimate([lx; ly]);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            % 计算预测测量与实际测量之间的误差，注意归一化角度误差
            pose = obj.edgeVertices{1}.x;      % 车辆位姿 [x; y; theta]
            landmark = obj.edgeVertices{2}.x;  % 地标位置 [l_x; l_y]
            dx = landmark(1) - pose(1);
            dy = landmark(2) - pose(2);
            r_pred = sqrt(dx^2 + dy^2);
            bearing_pred = atan2(dy, dx) - pose(3);
            % 归一化角度至 [-pi, pi]
            bearing_pred = atan2(sin(bearing_pred), cos(bearing_pred));
            h = [r_pred; bearing_pred];
            % 误差 = 测量值 - 预测值, 同时归一化角度误差
            error = obj.measurement - h;
            error(2) = atan2(sin(error(2)), cos(error(2)));
            obj.errorZ = error;
        end
        
        function linearizeOplus(obj)
            % linearizeOplus Compute the Jacobian of the error in the edge.
            % 计算误差函数对车辆状态和地标状态的雅可比矩阵
            pose = obj.edgeVertices{1}.x;      % 车辆位姿 [x; y; theta]
            landmark = obj.edgeVertices{2}.x;  % 地标位置 [l_x; l_y]
            dx = landmark(1) - pose(1);
            dy = landmark(2) - pose(2);
            q = dx^2 + dy^2;
            r = sqrt(q);
            
            % 防止除零，若 q 接近 0 则设定最小值
            if q < 1e-6
                q = 1e-6;
                r = sqrt(q);
            end
            
            % 对车辆状态 (vertex 1) 的雅可比 (注意负号)
            J1 = [ dx/r,    dy/r,    0;
                  -dy/q,   dx/q,    1 ];
              
            % 对地标状态 (vertex 2) 的雅可比 (注意负号)
            J2 = [ -dx/r,  -dy/r;
                    dy/q,   -dx/q ];
            
            obj.J{1} = J1;
            obj.J{2} = J2;
        end        
    end
end