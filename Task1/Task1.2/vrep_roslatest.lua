function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDroneBase')
    collection_handles= sim.getCollectionHandle('Obstacles')

    -- Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------

    start_handle = sim.getObjectHandle('Start')
    initial_waypoint_handle = sim.getObjectHandle('initial_waypoint')
    target_handle = sim.getObjectHandle('target')
    goal_1_handle = sim.getObjectHandle('goal_1')
    goal_2_handle = sim.getObjectHandle('goal_2')
 
    --Hint : Goal handles and other required handles
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------

    t = simOMPL.createTask('t')
    ss = {simOMPL.createStateSpace('6d', simOMPL.StateSpaceType.pose3d, drone_handle, {-2.97,-2.47,0.00},{3.02,2.52,2.95},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t, simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'), collection_handles})
    -- exchange edrone_visible with drone_handle
    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints

    path_flag=simROS.subscribe('/path_flag',  'std_msgs/Int16', 'set_path_flag') --Subscribed to '/path_flag' to get value of computed_path_flag


    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------


    scale_factor = {7.55, 7.56, 18.5} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required =  500  -- Add no of path points you want from one point to another
    compute_path_flag = -1
    whycon_z_ground_value = 55.60

end


function set_path_flag(num)
    compute_path_flag = num.data
    -- print ("flag_value:",num.data)
end

 
-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end


-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x =  -path[i]*scale_factor[1]
        pose.position.y =  -path[i+1]*scale_factor[2]
        pose.position.z =  whycon_z_ground_value - path[i+2]*scale_factor[3]
        sender.poses[math.floor(i/7) + 1] = pose
    
        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    --print sender.poses[0].position.x, sender.poses[0].position.y, sender.poses[0].position.z
    return sender
end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path

    r,path=simOMPL.compute(t,10.0,-1.0,no_of_path_points_required) -- Provide the correct arguments here.. Make sure you check the no of path points it actually computes
    -- print("path computed:", path)
    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        --print ("message to send:",message)
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end


function sysCall_actuation()
    
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    
    -- print ("in actuation()")

    if compute_path_flag >=0 then
        
	-- Condition for path planning from initial_waypoint to goal_1
        if compute_path_flag == 0 then
            -- print("in compute_path_flag == 1")
            -- Getting startpose
            start_pose = getpose(initial_waypoint_handle,-1)
            -- Getting the goalpose
            goal_pose = getpose(goal_1_handle,-1)
        end
    
	-- Condition for path planning from goal_1 to goal_2
        if compute_path_flag == 1 then
            -- print("in compute_path_flag == 1")
	        -- goal_1_handle gets the new start_pose
            start_pose = getpose(goal_1_handle,-1)
	        -- and goal_2_handle gets the new goal_pose
            goal_pose = getpose(goal_2_handle,-1)
        end

	-- Condition for path planning from goal_2 to intital_waypoint
        if compute_path_flag == 2 then        
	     -- goal_2_handle gets the new start_pose
            start_pose = getpose(goal_2_handle, -1)
	     -- and initial_waypoint_handle gets the new goal_pose
            goal_pose = getpose(initial_waypoint_handle, -1)
        end

    	-- Setting Start and Goal States 
        simOMPL.setStartState(t,start_pose)
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path here
        status = compute_and_send_path(t)
        if(status == true) then -- check if path is computed
            compute_path_flag = -1
        end
    end 

    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    -- print("getpose function: pose=", pose)
    return pose
end


function sysCall_sensing()

end



function sysCall_cleanup()

end
