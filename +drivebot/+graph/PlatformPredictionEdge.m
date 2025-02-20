classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % 计算初始估计：利用顶点1上的状态以及控制输入（存储于obj.z）
            % 计算顶点2的初始状态为： x2 = x1 + M * u ，其中
            %   M = dt * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]
            % 考虑实际时间戳（如果存在）
            
            % 获取顶点1状态
            v1 = obj.edgeVertices{1}.x;
            theta = v1(3);
            
            % 如果顶点有timestamp属性，则使用实际时间差，否则使用obj.dT
            if isprop(obj.edgeVertices{1}, 'timestamp') && isprop(obj.edgeVertices{2}, 'timestamp')
                dt = obj.edgeVertices{2}.timestamp - obj.edgeVertices{1}.timestamp;
            else
                dt = obj.dT;
            end
            
            % 构造旋转矩阵（仅2维部分）和整体预测矩阵
            R = [cos(theta), -sin(theta); 
                 sin(theta),  cos(theta)];
            M = [dt * R, [0; 0];
                 0, 0, dt];
            
            % 利用控制输入 (存于obj.z) 进行初始预测
            obj.edgeVertices{2}.x = v1 + M * obj.z;
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % 误差根据公式计算:
            %   e = inv(M) * (x2 - x1) - u
            % 其中 M = dt * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 dt]
            % 从顶点1的状态中提取theta，若顶点有timestamp属性则使用实际的dt
            
            % 获取顶点状态
            v1 = obj.edgeVertices{1}.x;
            v2 = obj.edgeVertices{2}.x;
            
            if isprop(obj.edgeVertices{1}, 'timestamp') && isprop(obj.edgeVertices{2}, 'timestamp')
                dt = obj.edgeVertices{2}.timestamp - obj.edgeVertices{1}.timestamp;
            else
                dt = obj.dT;
            end
            
            theta = v1(3);
            
            % 对于前两维构造旋转矩阵和其逆（利用旋转矩阵的转置）
            R = [cos(theta), -sin(theta); 
                 sin(theta),  cos(theta)];
            R_inv = [cos(theta), sin(theta); 
                     -sin(theta), cos(theta)];
            
            % 构造M的逆：对前两维为 (1/dt)*R_inv, 第三维为1/dt
            invM = [1/dt * R_inv, [0;0]; 
                    0, 0, 1/dt];
                
            % 计算状态差分
            diff = v2 - v1;
            
            % 得到误差：若预测与实际匹配则e应为0
            error = invM * diff - obj.z;
            obj.errorZ = error;
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % 根据误差 e = inv(M) * (x2 - x1) - u 求雅可比，其中 M依赖于顶点1的theta
            % 对顶点2，导数为 inv(M)；对顶点1不仅有 -inv(M) ，还需考虑inv(M)关于theta的导数.
            
            % 获取顶点状态
            v1 = obj.edgeVertices{1}.x;
            v2 = obj.edgeVertices{2}.x;
            
            if isprop(obj.edgeVertices{1}, 'timestamp') && isprop(obj.edgeVertices{2}, 'timestamp')
                dt = obj.edgeVertices{2}.timestamp - obj.edgeVertices{1}.timestamp;
            else
                dt = obj.dT;
            end
            
            delta = v2 - v1;
            theta = v1(3);
            c = cos(theta);
            s = sin(theta);
            
            % 对顶点2雅可比：直接等于inv(M)
            J2 = zeros(3, 3);
            J2(1:2, 1:2) = (1/dt) * [c, s; -s, c];
            J2(3, 3) = 1/dt;
            
            % 对顶点1雅可比
            J1 = zeros(3, 3);
            J1(1:2, 1:2) = - (1/dt) * [c, s; -s, c];
            % 计算R_inv对theta的导数, dR_inv/dtheta = [-s, c; -c, -s]
            dRinv_dtheta = [-s, c; -c, -s];
            J1(1:2, 3) = (1/dt) * dRinv_dtheta * (delta(1:2));
            J1(3, 3) = -1/dt;
            
            obj.J{1} = J1;
            obj.J{2} = J2;
        end
    end    
end