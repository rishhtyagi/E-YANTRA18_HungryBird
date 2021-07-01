-- This script is used for realtime emulation of the environment in V-REP
'''
*Team Id : eYRC#3589
*Author List : Madhav Sharma, Rishabh Tyagi, Bharat Jain, Apoorv Kotnala
*Filename : 
*Theme : Hungry Bird 
*Functions : sysCall_init ,sysCall_actuation, sysCall_sensing, sysCall_cleanup, aruco_callback, whycon_callback, key_callback
*Global Variables : position_hoop1, position_hoop3, position_obstacle1, orientation_hoop1, orientation_hoop3, whycon_ground_z_value, real_whycon_positions, whycon_to_vrep_scale

'''
function sysCall_init()

	-- Creating handles for position_hoop1, position_hoop3, position_obstacle1
    position_hoop1_handle = sim.getObjectHandle('Position_hoop1')
    position_hoop3_handle = sim.getObjectHandle('Position_hoop3')
    position_obstacle1_handle = sim.getObjectHandle('obstacle_1')

    -- Creating handles for orientation_hoop1, orientation_hoop3
    orientation_hoop1_handle = sim.getObjectHandle('Orientation_hoop1')
    orientation_hoop3_handle = sim.getObjectHandle('Orientation_hoop3')

    -- position_hoop1: Gets position of hoop1 
    position_hoop1 = sim.getObjectPosition(position_hoop1_handle, -1)
    -- position_hoop3: Gets postion of hoop3
    position_hoop3 = sim.getObjectPosition(position_hoop3_handle, -1)
    -- position_obstacle1: Gets position of obstacle1
    position_obstacle1 = sim.getObjectPosition(position_obstacle1_handle, -1)

    -- orientation_hoop1: Gets position of hoop1
    orientation_hoop1 = sim.getObjectQuaternion(orientation_hoop1_handle, -1)
    -- orientation_hoop3: Gets position of hoop3
    orientation_hoop3 = sim.getObjectQuaternion(orientation_hoop3_handle, -1)
    
	-- whycon_ground_z_value: value of whycon's z coordinate when it is kept at ground    
   	whycon_ground_z_value = 33

    -- whycon_to_vrep_scale: Stores scale factors calculated in each axis
    whycon_to_vrep_scale = {nil,nil,nil}

    -- Subscribing to the /aruco_marker_publisher/markers 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    -- Subscribing to the /whycon/poses
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    -- Subscribing to the /input_key
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function aruco_callback(msg)
    -- aruco_orientation1: List to store values of orientation to be given to orientation_hoop1
       aruco_orientation1 = {msg.markers[2].pose.pose.orientation.x, msg.markers[2].pose.pose.orientation.y, msg.markers[2].pose.pose.orientation.z+5, msg.markers[2].pose.pose.orientation.w-5}

    -- aruco_orientation3: List to store values of orientation to be given to orientation_hoop3
    aruco_orientation3 = {msg.markers[1].pose.pose.orientation.x, msg.markers[1].pose.pose.orientation.y, -msg.markers[1].pose.pose.orientation.z, -msg.markers[1].pose.pose.orientation.w}


end

function whycon_callback(msg)
    
    -- w_0: Coordinates for whycon of index0
    w_0 = {msg.poses[1].position.x, msg.poses[1].position.y, msg.poses[1].position.z}
    -- w_1: Coordinates for whycon of index1
    w_1 = {msg.poses[2].position.x, msg.poses[2].position.y, msg.poses[2].position.z}
    -- w_2: Coordinates for whycon of index2
    w_2 = {msg.poses[3].position.x, msg.poses[3].position.y, msg.poses[3].position.z}
    
    -- Checking quadrant of whycons obtained
    if w_0[1] < 0 and w_0[2] < 0 then
        whycon_0 = w_0
        if w_1[1] > 0 then
            whycon_1 = w_1
            whycon_2 = w_2
        else
            whycon_1 = w_2
            whycon_2 = w_1
        end
    elseif w_1[1] < 0 and w_1[2] < 0 then
        whycon_0 = w_1
        if w_0[1] > 0 then
            whycon_1 = w_0
            whycon_2 = w_2
        else
            whycon_1 = w_2
            whycon_2 = w_0
        end
    else 
        whycon_0 = w_2
        if w_1[1] > 0 then
            whycon_1 = w_1
            whycon_2 = w_0
        else
            whycon_1 = w_0
            whycon_2 = w_1
        end
    end

    -- -- h1: Gets position of hoop1 
    -- h1 = sim.getObjectPosition(position_hoop1_handle, -1)
    -- -- h3: Gets position of hoop3 
    -- h3 = sim.getObjectPosition(position_hoop3_handle, -1)
    -- -- 01: Gets position of obstacle1 
    -- o1 = sim.getObjectPosition(position_obstacle1_handle, -1)

    -- Scale factors are calculated for hoop1 for each axis 
    whycon_to_vrep_scale_h1x = ((position_hoop1[1] - 0.0)/(whycon_0[1] - 0.0))
    whycon_to_vrep_scale_h1y = ((position_hoop1[2] - 0.0)/(whycon_0[2] - 0.0))
    whycon_to_vrep_scale_h1z = ((position_hoop1[3] - 0.0)/(-whycon_0[3] + whycon_ground_z_value))

    -- Scale factors are calculated for hoop3 for each axis  
    whycon_to_vrep_scale_h3x = ((position_hoop3[1] - 0.0)/(whycon_1[1] - 0.0))
    whycon_to_vrep_scale_h3y = ((position_hoop3[2] - 0.0)/(whycon_1[2] - 0.0))
    whycon_to_vrep_scale_h3z = ((position_hoop3[3] - 0.0)/(-whycon_1[3] + whycon_ground_z_value))

    -- Scale factors are calculated for obstacle1 for each axis
    whycon_to_vrep_scale_o1x = ((position_obstacle1[1] - 0.0)/(whycon_2[1] - 0.0))
    whycon_to_vrep_scale_o1y = ((position_obstacle1[2] - 0.0)/(whycon_2[2] - 0.0))
    whycon_to_vrep_scale_o1z = ((position_obstacle1[3] - 0.0)/(-whycon_2[3] + whycon_ground_z_value))

    -- whycon_to_vrep_scale: List to store calculated scale factors
    whycon_to_vrep_scale[1] = (whycon_to_vrep_scale_h1x + whycon_to_vrep_scale_h3x + whycon_to_vrep_scale_o1x)/3
    whycon_to_vrep_scale[2] = (whycon_to_vrep_scale_h1y + whycon_to_vrep_scale_h3y + whycon_to_vrep_scale_o1y)/3
    whycon_to_vrep_scale[3] = (whycon_to_vrep_scale_h1z + whycon_to_vrep_scale_h3z + whycon_to_vrep_scale_o1z)/3

    -- Calculating x ,y ,z values of positions for hoop1 in vrep_world frame 
    vrep_value1 = {-(whycon_0[1]*whycon_to_vrep_scale[1]), -(whycon_0[2]*whycon_to_vrep_scale[2]), h1[3]}
    -- Calculating x ,y ,z values of positions for hoop2 in vrep_world frame 
    vrep_value2 = {-(whycon_1[1]*whycon_to_vrep_scale[1]), -(whycon_1[2]*whycon_to_vrep_scale[2]), h3[3]}
    -- Calculating x ,y ,z values of positions for hoop3 in vrep_world frame 
    vrep_value3 = {-(whycon_2[1]*whycon_to_vrep_scale[1]), -(whycon_2[2]*whycon_to_vrep_scale[2]), o1[3]}

    -- vrep_transformed_position: List of positions of hoops and obstacle in vrep_world_frame
    vrep_transformed_position = {vrep_value1, vrep_value2, vrep_value3}

end

function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
   
    -- Press ' / ' to set the positions and orientations of hoops
    if msg.data == 500 then
        -- Set object positions for hoop1, hoop3, obstacle1 using positions_hoop1_handles, positions_hoop3_handles, position_obstacle1_handle
        sim.setObjectPosition(position_hoop1_handle, -1, vrep_transformed_position[1])
        sim.setObjectPosition(position_hoop3_handle, -1, vrep_transformed_position[2])
        sim.setObjectPosition(position_obstacle1_handle, -1, vrep_transformed_position[3])

        -- Set object orientations for hoop1, hoop3 using orientation_hoop1_handle, orientation_hoop3_handle
        sim.setObjectQuaternion(orientation_hoop1_handle ,-1, aruco_orientation1)
        sim.setObjectQuaternion(orientation_hoop3_handle ,-1, aruco_orientation3) 
    end
   
    -- Press ' 0 ' to set the positions and orientations of hoops
    if msg.data == 600 then
        -- Set initial object positions for hoop1, hoop3, obstacle1 using positions_hoop1_handles, positions_hoop3_handles, position_obstacle1_handle
    	sim.setObjectPosition(position_hoop1_handle, -1, position_hoop1)
    	sim.setObjectPosition(position_hoop3_handle, -1, position_hoop3)
        sim.setObjectPosition(position_obstacle1_handle, -1, position_obstacle1)

         -- Set initial object orientations for hoop1, hoop3 using orientation_hoop1_handle, orientation_hoop3_handle
        sim.setObjectQuaternion(orientation_hoop1_handle ,-1, orientation_hoop1) 
        sim.setObjectQuaternion(orientation_hoop3_handle ,-1, orientation_hoop3)    
    end
end
