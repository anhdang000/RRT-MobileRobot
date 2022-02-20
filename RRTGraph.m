classdef RRTGraph
    % RRT algorithm
    
    properties
        % Map
        start
        goal
        mapMatrix
        mapW
        mapH
        
        % Tree
        treeCoors
        parent = 0
        
        % Path
        goalState
        goalFlag = 0
        path = []
        
        dMax = 0.3
    end
    
    methods
        function obj = RRTGraph(start, goal, mapMatrix, mapSize)
            obj.start = start;
            obj.goal = goal;
            obj.mapMatrix = mapMatrix;
            obj.mapW = mapSize(1);
            obj.mapH = mapSize(2);
            obj.treeCoors = start;
            
        end
        
        function obj = addNode(obj, n, node)
            obj.treeCoors = insertElement(obj.treeCoors, node, n);
        end
        
        function obj = removeNode(obj, n)
            obj.treeCoors = popElement(obj.treeCoors, n);
        end
        
        function obj = addEdge(obj, parent, child)
            obj.parent = insertElement(obj.parent, parent, child);
        end
        
        function obj = removeEdge(obj, child)
            obj.parent = popElement(obj.parent, child);
        end
        
        function numNodes = getNumNodes(obj)
            numNodes = size(obj.treeCoors, 1);
        end
        
        function distance = getDistance(obj, n1, n2)
            distance = norm(obj.treeCoors(n1, :) - obj.treeCoors(n2, :));
        end
        
        function nodeNew = sampleEnv(obj)
            x = rand*obj.mapW;
            y = rand*obj.mapH;
            nodeNew = [x, y];
        end
        
        function nNear = getNearest(obj, n)
            dMin = getDistance(obj, 1, n);
            nNear = 1;
            for i = 1:n-1
                if getDistance(obj, i, n) < dMin
                    dMin = getDistance(obj, i, n);
                    nNear = i;
                end
            end
        end
        
        function [obj, flag] = isFree(obj)
            % flag = 1 when the node is valid (NOT collide with any obstacles)
            n = getNumNodes(obj);
            lastNode = obj.treeCoors(n, :);
            checkMat = zeros(size(obj.mapMatrix));
            try
            checkMat(ceil(lastNode(2)*100), ceil(lastNode(1)*100)) = 1;
            catch
                disp(lastNode);
            end
            flag = ~max(checkMat .* obj.mapMatrix, [], 'all');
            
            if ~flag
                obj = removeNode(obj, n);
            end
        end
        
        function boolVal = isCrossObstacles(obj, node1, node2)
            checkMat = zeros(size(obj.mapMatrix));
            checkMat = insertShape(checkMat, 'Line', round([node1*100 node2*100]), ...
                'LineWidth', 2, 'Color', 'white');
            checkMat = checkMat(:, :, 1);
            boolVal = max(checkMat .* obj.mapMatrix, [], 'all');
        end
        
        function obj = connect(obj, n1, n2)
            node1 = obj.treeCoors(n1, :);
            node2 = obj.treeCoors(n2, :);
            if isCrossObstacles(obj, node1, node2)
                obj = removeNode(obj, n2);
            else
                obj = addEdge(obj, n1, n2);
            end
        end
        
        function obj = step(obj, nNear, nRand)
            d = getDistance(obj, nNear, nRand);
            if d > obj.dMax
                nodeNear = obj.treeCoors(nNear, :);
                nodeRand = obj.treeCoors(nRand, :);
                p = nodeRand - nodeNear;
                theta = atan2(p(2), p(1));
                nodeNew = nodeNear + obj.dMax * [cos(theta) sin(theta)];
                obj = removeNode(obj, nRand);
                if abs(nodeNew - obj.goal) < obj.dMax == [1, 1]
                    % Reach goal, add goal node
                    obj = addNode(obj, nRand, obj.goal);
                    obj.goalState = nRand;
                    obj.goalFlag = 1;
                else
                    % Add normal node
                    obj = addNode(obj, nRand, nodeNew);
                end
            end
        end
        
        function obj = getPath2Goal(obj)
            if obj.goalFlag
                obj.path = obj.goalState;
                try
                    newPos = obj.parent(obj.goalState);
                    while newPos ~= 1
                        obj.path = [obj.path; newPos];
                        newPos = obj.parent(newPos);
                    end
                    obj.path = [obj.path; 1];
                catch
                    obj.goalFlag = 0;
                end
            end
        end
        
        function pathCoors = getPathCoors(obj)
            pathCoors = [];
            for i = 1:length(obj.path)
                n = obj.path(i);
                node = obj.treeCoors(n, :);
                pathCoors = [pathCoors; node];
            end
            pathCoors = flip(pathCoors, 1);
        end
        
        function optimalPathCoors = optimizePath(obj, pathCoors)
            numPoints = size(pathCoors, 1);
            
            firstIdx = 1;
            optimalPathCoors = pathCoors(firstIdx, :);
            while 1
                secondIdx = firstIdx+1;
                while secondIdx <= numPoints
                    p1 = pathCoors(firstIdx, :);
                    p2 = pathCoors(secondIdx, :);
                    checkMat = zeros(size(obj.mapMatrix));
                    checkMat = insertShape(checkMat, 'Line', round([p1*100 p2*100]), ...
                        'LineWidth', 2, 'Color', 'white');
                    checkMat = checkMat(:, :, 1);
                    isCrossObstacles = max(checkMat .* obj.mapMatrix, [], 'all');
                    if isCrossObstacles
                        secondIdx = secondIdx - 1;
                        firstIdx = secondIdx;
                        optimalPathCoors = [optimalPathCoors; pathCoors(secondIdx, :)];
                        break
                    elseif secondIdx == numPoints
                        optimalPathCoors = [optimalPathCoors; pathCoors(secondIdx, :)];
                        break
                    else
                        secondIdx = secondIdx + 1;
                    end
                end
                if secondIdx == numPoints
                    break
                end
            end
        end
        
        function obj = bias(obj, nodeGoal)
            n = getNumNodes(obj);
            obj = addNode(obj, n+1, nodeGoal);
            nNear = getNearest(obj, n+1);
            obj = step(obj, nNear, n+1);
            obj = connect(obj, nNear, n+1);
        end
        
        function obj = expand(obj)
            n = getNumNodes(obj);
            nodeNew = sampleEnv(obj);
            obj = addNode(obj, n+1, nodeNew);
            [obj, flag] = isFree(obj);
            if flag
                nNear = getNearest(obj, n+1);
                obj = step(obj, nNear, n+1);
                obj = connect(obj, nNear, n+1);
            end
        end
    end
end

