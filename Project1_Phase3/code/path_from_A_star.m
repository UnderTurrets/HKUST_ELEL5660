function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    
    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording whether a node is expanded (removed from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3);
    
    %Start your code here ==================================================================
    % TODO: visit the start node. For example,
    g(xStart, yStart, zStart) = 0;
    h = abs(xStart - xTarget) + abs(yStart - yTarget) + abs(zStart - zTarget);
    f = g(xStart, yStart, zStart) + h;
    queue = [queue; [xStart, yStart, zStart, f]];
    
    % TODO: Expansion Loop (Refer to the slide)
    while true
        % TODO: Check if the queue is emtpy. If empty, terminate the loop.
        if (size(queue,1)==0)
            break;
        end
        % TODO: visit (remove) the node with the smalled f score (f = g +
        % h).
        [~, idx] = min(queue(:, 4));
        % Current node
        current = queue(idx, 1:3);
        % Remove from queue
        queue(idx, :) = [];
        
        % TODO: Check if the visited node is the target. If yes, terminate
        % the loop
        if isequal(current, [xTarget, yTarget, zTarget])
            break;
        end
        
        % TODO: Expand towards 6 directions (x-1, x+1, y-1, y+1, z-1, z+1)
        directions = [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1];
        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);
            x = neighbor(1);
            y = neighbor(2);
            z = neighbor(3);
            % Check bounds and obstacles
            if x > 0 && x <= MAX_X && y > 0 && y <= MAX_Y && z > 0 && z <= MAX_Z && MAP(x, y, z) ~= -1
                % Calculate g score
                tentative_g = g(current(1), current(2), current(3)) + 1; 
                if tentative_g < g(x, y, z)
                    % This path to neighbor is better than any previous one
                    g(x, y, z) = tentative_g;
                    % Recalculate heuristic
                    h = abs(x - xTarget) + abs(y - yTarget) + abs(z - zTarget);
                    f = g(x, y, z) + h;
                    % Add to queue
                    queue = [queue; [x, y, z, f]];
                    % Record parent
                    parents(x, y, z, :) = current; 
                end
            end
        end
    end
    
    % TODO: retrace the optimal path
    cur_pos = [xTarget yTarget zTarget];
    Optimal_path = [xTarget yTarget zTarget];
    while ~isequal(cur_pos,[xStart yStart zStart])
        % TODO: get the parent node
        cur_pos = squeeze(parents(cur_pos(1), cur_pos(2), cur_pos(3), :))';
        
        % stack the parent node into Optimal_path
        Optimal_path = [cur_pos ;Optimal_path];
    end
    % Let's use the centre position of each node
    Optimal_path = Optimal_path - 0.5;
end
