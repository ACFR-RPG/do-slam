% function [obj] = LMSolver(obj,config,graph0,measurementsCell,system)
function [obj] = LMSolver(obj,config,graph0,measurementsCell)
%LM solver combines gradient descent and gauss-newton

done      = 0;
iteration = 1;
lambda     = 1e-8; %initial value
lambdaUp   = 8;
lambdaDown = 12;
system = System(config,graph0,measurementsCell);

% covariance = system.covariance; %doesn't change in this function
errorCurrent = norm(system.b);   %initial value
weightCurrent = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isempty(config.robustCostFunction)
    errorNorm = zeros(graph0.nEdges,1);
    for i=1:graph0.nEdges
        switch config.robustCostFunction
%             case 'cauchy'
%                 weightedErrorNorm = cauchyRobustCostFunction(config,system,i);
%             case 'DCS'
%                 weightedErrorNorm = DCSRobustCostFunction(config,system,i);
%             case 'fair'
%                 weightedErrorNorm = FairRobustCostFunction(config,system,i);
            case 'gemanMcClure'
                weightedErrorNorm = GemanMcClureRobustCostFunction(config,system,i);
%             case 'huber'
%                 weightedErrorNorm = HuberRobustCostFunction(config,system,i);
            case 'pseudoHuber'
                weightedErrorNorm = PseudoHuberRobustCostFunction(config,system,i);
%             case 'saturated'
%                 weightedErrorNorm = SaturatedRobustCostFunction(config,system,i);
%             case 'tukey'
%                 weightedErrorNorm = TukeyRobustCostFunction(config,system,i);
%             case 'welsch'
%                 weightedErrorNorm = WelschRobustCostFunction(config,system,i);
            otherwise % default --> pseudo-Huber
                weightedErrorNorm = PseudoHuberRobustCostFunction(config,system,i);
        end
        errorNorm(i) = weightedErrorNorm;
    end
    weightedErrorCurrent = norm(errorNorm);
    weightCurrent = weightedErrorCurrent/errorCurrent;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%storing variables from each iteration
obj.dX      = [];
obj.graphs  = [graph0];
obj.systems = [system];

timeStart = tic;
while (~done)
    system.b = weightCurrent*system.b;
    %   build system & solve    
   if strcmp(config.processing,'incrementalSolveCholesky')
    d = spdiags(lambda*spdiags(system.L,0),0,size(system.L,1),size(system.L,2));
    dX = (system.L + d) \ system.d;
   else
    d = spdiags(lambda*spdiags(system.H,0),0,size(system.H,1),size(system.H,2));
    dX = (system.H + d) \ system.c;
   end
    
    if strcmp(config.planeNormalParameterisation,'S2')
        dX = system.kPerp*dX;   
        dX = adjustUpdate(system,graph0,dX);
    end
    
    %update system, get error
    graph0Update = graph0.updateVertices(config,system,dX);
    graph1 = graph0Update.updateEdges(config);
    errorTemp = norm(graph1.constructResiduals(config,measurementsCell));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(config.robustCostFunction)
        errorTempNorm = zeros(graph1.nEdges,1);
        systemUpdate = System(config,graph1,measurementsCell);
        for i=1:graph0.nEdges
            switch config.robustCostFunction
%                 case 'cauchy'
%                     weightedErrorTempNorm = cauchyRobustCostFunction(config,systemUpdate,i);
%                 case 'DCS'
%                     weightedErrorTempNorm = DCSRobustCostFunction(config,systemUpdate,i);
%                 case 'fair'
%                     weightedErrorTempNorm = FairRobustCostFunction(config,systemUpdate,i);
                case 'gemanMcClure'
                    weightedErrorTempNorm = GemanMcClureRobustCostFunction(config,systemUpdate,i);
%                 case 'huber'
%                     weightedErrorTempNorm = HuberRobustCostFunction(config,systemUpdate,i);
                case 'pseudoHuber'
                    weightedErrorTempNorm = PseudoHuberRobustCostFunction(config,systemUpdate,i);
%                 case 'saturated'
%                     weightedErrorTempNorm = SaturatedRobustCostFunction(config,systemUpdate,i);
%                 case 'tukey'
%                     weightedErrorTempNorm = TukeyRobustCostFunction(config,systemUpdate,i);
%                 case 'welsch'
%                     weightedErrorTempNorm = WelschRobustCostFunction(config,systemUpdate,i);
                otherwise % default --> pseudo-Huber
                    weightedErrorTempNorm = PseudoHuberRobustCostFunction(config,systemUpdate,i);
            end
            errorTempNorm(i) = weightedErrorTempNorm;
        end
        weightedErrorTemp = norm(errorTempNorm);
        weightTemp = weightedErrorTemp/errorTemp;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %can compute chi-squared error with covariance and residuals
    %b = graph1.constructResiduals(measurementsCell);
    %chiSquared = (b'/covariance)*b;
    %chiSquaredErrorTemp = chiSquared/graph1.nEdges;
    
    % update
    rho = errorCurrent - errorTemp;
    if rho > 0 %(ie if errorTemp < errorCurrent -> error decreasing)
        %use update
        lambda = lambda/lambdaDown;
        errorCurrent = errorTemp;
        graph0 = graph1;
        system = System(config,graph1,measurementsCell);
        updateGraph = 1;
        if ~isempty(config.robustCostFunction)
            weightCurrent = weightTemp;
        end
    else
        %dont use update
        lambda = lambda*lambdaUp;
        updateGraph = 0;
    end
    
    %display progress
    if config.displayProgress %&& updateGraph
        display(sprintf('It. %d:\t||dX|| = %.3e\t,\trho = %.3e\t,\tlambda = %.3e',iteration,norm(dX),rho,lambda))
    end
    
    %   check termination criteria
%     if (rho > 0 && norm(rho) < 1e-8) || (iteration >= obj.maxIterations) || norm(dX) > config.maxNormDX
%       graph0 = graph1; %more careful, but slow
    if (norm(dX) <= obj.threshold) || (iteration >= obj.maxIterations) || norm(dX) > config.maxNormDX ...
            || (lambda == 0)
        done = 1;
    else
        %   increment interation
        if updateGraph
            iteration = iteration + 1;
            %store
%             obj.dX = [obj.dX dX];
%             obj.graphs = [obj.graphs graph0];
%             obj.systems = [obj.systems system];
            % to save memory
                obj.dX = dX;
                obj.graphs = graph0;
                obj.systems = system;
        end
    end
    
    % to save graph files after every iteration -- added as a quick way to 
    % solve out of memory errors (re-run starting last graph file obtained 
    % until you reach a solution)
%     t = datetime('now');
%     secondStr = num2str(t.Second);
%     graph0.saveGraphFile(config,strcat('results_',num2str(t.Hour),...
%         '_',num2str(t.Minute),'_',secondStr(1:2),'.graph'))
end

obj.solveTime = toc(timeStart);
obj.iterations = iteration;

end