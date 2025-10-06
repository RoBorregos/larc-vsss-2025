import math

def select_robot(past_robots, detected_robot, tf_helper, get_eucladian):
    '''
    Function to get the robots id's based on the last frame detections
    using aspects such as: relative distance (now -> past), team, id, secondary color.
    
    Should be called for both color teams (yellow and blue)
    
    Returns a list of the detected robots in a frame
    '''
    robot_list = []
    #compare the actual robot with the list of past robots;
    possibilities = []
    same_id = []
    same_color = []
    same_team = []
    not_same = []

    for past_robot in past_robots:
        distance = get_eucladian(detected_robot.location, past_robot.location) #check number sign, if negative, use abs
        detected_robot.relative_distance = distance
        if detected_robot.id == past_robot.id:
            same_id.append(detected_robot)
        elif detected_robot.secondary_color == past_robot.secondary_color:
            same_color.append(detected_robot)
        elif detected_robot.team == past_robot.team:
            same_team.append(detected_robot)
        else:
            not_same.append(detected_robot)

    same_id_ordered = sorted(same_id, key=lambda r:r.relative_distance) #check if this focus is correct, appending the four hole arrays?
    same_color_ordered = sorted(same_color, key=lambda r:r.relative_distance)
    same_team_ordered = sorted(same_team, key=lambda r:r.relative_distance)
    not_same_ordered = sorted(not_same, key=lambda r:r.relative_distance)
    possibilities.append(same_id_ordered)
    possibilities.append(same_color_ordered)
    possibilities.append(same_team_ordered)
    possibilities.append(not_same_ordered)

    selected_robot = None
    for possibility in possibilities:
        if len(possibility) > 0:
            selected_robot = possibility[0]
            break

    if selected_robot.id == None:
        selected_robot.id = past_robot.id
    if selected_robot.secondary_color == None:
        selected_robot.secondary_color = past_robot.secondary_color
    if selected_robot.team == None:
        selected_robot.team = past_robot.team
    if selected_robot.angle == None:
        selected_robot.angle = past_robot.angle
    
    if selected_robot is not None:
        robot_list.append(selected_robot)

    yaw = math.radians(selected_robot.angle)
    pitch, roll = 0.0, math.pi
    tf_helper("robot" + str(selected_robot.id) + "_base_link", selected_robot.location[0], selected_robot.location[1], roll, pitch, yaw)
    
    return robot_list