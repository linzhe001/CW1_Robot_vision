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
            % Compute the initial estimate: Using the state at vertex 1 and 
            % the control input (stored in obj.z), the initial state at vertex 2 
            % is computed as: x2 = x1 + M * u, where
            % M = dt * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]
            % The actual timestamps are considered if available.
            
            % Retrieve the state at vertex 1
            v1 = obj.edgeVertices{1}.x;
            theta = v1(3);
            
            % If the vertices have a timestamp property, use the actual time difference;
            % otherwise, use obj.dT.
            if isprop(obj.edgeVertices{1}, 'timestamp') && isprop(obj.edgeVertices{2}, 'timestamp')
                dt = obj.edgeVertices{2}.timestamp - obj.edgeVertices{1}.timestamp;
            else
                dt = obj.dT;
            end
            
            % Construct the rotation matrix (only for the 2D part) and 
            % the overall prediction matrix.
            R = [cos(theta), -sin(theta); 
                 sin(theta),  cos(theta)];
            M = [dt * R, [0; 0];
                 0, 0, dt];
            
            % Use the control input (stored in obj.z) for the initial prediction.
            obj.edgeVertices{2}.x = v1 + M * obj.z;
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % The error is computed using the formula:
            %   e = inv(M) * (x2 - x1) - u
            % where M = dt * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 dt]
            % Extract theta from the state at vertex 1. If the vertices have 
            % a timestamp property, use the actual dt.
            
            % Retrieve the state of the vertices
            v1 = obj.edgeVertices{1}.x;
            v2 = obj.edgeVertices{2}.x;
            
            % If the vertices have a timestamp property, use the actual time difference;
            % otherwise, use obj.dT.
            if isprop(obj.edgeVertices{1}, 'timestamp') && isprop(obj.edgeVertices{2}, 'timestamp')
                dt = obj.edgeVertices{2}.timestamp - obj.edgeVertices{1}.timestamp;
            else
                dt = obj.dT;
            end
            
            theta = v1(3);
            
            % Construct the rotation matrix for the first two dimensions
            % and its inverse (using the transpose of the rotation matrix)
            R = [cos(theta), -sin(theta); 
                 sin(theta),  cos(theta)];
            R_inv = [cos(theta), sin(theta); 
                     -sin(theta), cos(theta)];
            
            % Construct the inverse of M: For the first two dimensions, it is (1/dt) * R_inv;
            % for the third dimension, it is 1/dt.
            invM = [1/dt * R_inv, [0;0]; 
                    0, 0, 1/dt];
                
            % Compute the state difference
            diff = v2 - v1;
            
            % Compute the error: If the prediction matches the actual value, e should be 0.
            error = invM * diff - obj.z;
            obj.errorZ = error;
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Compute the Jacobians based on the error formula:
            %   e = inv(M) * (x2 - x1) - u
            % where M depends on theta at vertex 1.
            % 
            % For vertex 2, the derivative is simply inv(M).
            % For vertex 1, in addition to -inv(M), the derivative 
            % of inv(M) with respect to theta must also be considered.
            
            % Retrieve the state of the vertices
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
            
            % Jacobian for vertex 2: Directly equal to inv(M)
            J2 = zeros(3, 3);
            J2(1:2, 1:2) = (1/dt) * [c, s; -s, c];
            J2(3, 3) = 1/dt;
            
            % Jacobian for vertex 1
            J1 = zeros(3, 3);
            J1(1:2, 1:2) = - (1/dt) * [c, s; -s, c];
            % Compute the derivative of R_inv with respect to theta:
            % dR_inv/dtheta = [-s, c; -c, -s]
            dRinv_dtheta = [-s, c; -c, -s];
            J1(1:2, 3) = (1/dt) * dRinv_dtheta * (delta(1:2));
            J1(3, 3) = -1/dt;
            
            obj.J{1} = J1;
            obj.J{2} = J2;
        end
    end    
end