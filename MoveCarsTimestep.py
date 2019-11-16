import math
import matplotlib


def rotate(x, y, theta):
    R =[[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]
    P =[x, y]
    return R * P

def draw_rectangle(center, theta, height, width, color, edge_color, display_option):
    x = center(1)
    y = center(2)
    x_v = [x ,x + height, x + height, x, x]
    y_v = [y  , y, y + width , y + width, y]
    # rotate angle theta
    R = []
    R[1,:]=x_v - x
    R[2,:]=y_v - y
    XY = [math.cos(theta) - math.sin(theta), math.sin(theta), math.cos(theta)]*R;
    XY[1,:] = XY[1,:] + x
    XY[2,:] = XY[2,:] + y
    R = rotate(height / 2, width / 2, theta)

    X = XY[1,:] - R[1]
    Y = XY[2,:] - R[2]

    RectLines = [[X[1],Y[1],X[2],Y[2]],[X[2],Y[2],X[3],Y[3]], [X[3],Y[3],X[4],Y[4]],[X[4],Y[4],X[5],Y[5]]]

    if (display_option):
        rect = matplotlib.fill(X, Y, color)
        set(rect, 'FaceColor', color, 'EdgeColor', edge_color, 'LineWidth', 1)

# Move Car and Draw Environment - Get Sensor Readings and Collision State
def MoveCarsTimestep(newCenters, sensor_readings, carLines, collision_bools,carLocations, carHeadings, prev_carLines,steerAngles, car, sensor, env, display_option):
    # Intializations
    sensor_readings = [[0 for col in range(len(sensor.angles))] for row in range(len(carHeadings))]# row for each car
    sensor_lines = [1, len(carHeadings)]
    carLines = [1, len(carHeadings)]
    sensor.angles = sensor.angles - math.pi / 2
    collision_bools = [[0 for col in range(len(carHeadings))] for row in range(1)]

    # Colors Configurations
    car_outer_color = [0,0,0]
    car_inner_color = [1,1,0]
    car_wheels_color = [0,0,0]
    sensor_beam_color = [1,0,0]

    # Draw Environment
    if (display_option == 1 or display_option == 2):
        for i in range(1,len(env.lines[:, 1])):
            matplotlib.line([env.lines(i, 1),env.lines(i, 3)], [env.lines(i, 2),env.lines(i, 4)])

    #Draw Destinations
    if (display_option == 1 or display_option == 2):
        matplotlib.plot(env.destination(1), env.destination(2), 'r.-', 'markersize',10 * env.destination_dot_radius_ratio * car.width)

    for car_id in range(1,len(carHeadings)):
    #Draw car
        carCentre(1) = carLocations(car_id, 1) - (car.length / 2) * math.cos(carHeadings(car_id))
        carCentre(2) = carLocations(car_id, 2) - (car.length / 2) * math.sin(carHeadings(car_id))
        theta = carHeadings(car_id)
        carLines[car_id] = draw_rectangle(carCentre, theta, car.length, car.width, car_inner_color, car_outer_color, display_option)

        #Write Car Number
        if (display_option == 1 or  display_option == 2):
            matplotlib.text(carCentre(1), carCentre(2), str(car_id))

        #Draw Four Wheels
        if (display_option == 1):
            newCenters = rotate(car.wheelBase / 2, car.width / 2, carHeadings(car_id))
            newCenters = newCenters + carCentre
            theta = carHeadings(car_id) + steerAngles(car_id)
            draw_rectangle(newCenters, theta, car.wheelLength, car.wheelWidth, car_wheels_color, car_wheels_color,display_option)

            newCenters = rotate(car.wheelBase / 2, -car.width / 2, carHeadings(car_id))
            newCenters = newCenters + carCentre
            theta = carHeadings(car_id) + steerAngles(car_id)
            draw_rectangle(newCenters, theta, car.wheelLength, car.wheelWidth, car_wheels_color, car_wheels_color,display_option)

            newCenters = matplotlib.rotate(-car.wheelBase / 2, car.width / 2, carHeadings(car_id))
            newCenters = newCenters + carCentre
            draw_rectangle(newCenters, carHeadings(car_id), car.wheelLength, car.wheelWidth, car_wheels_color, car_wheels_color,display_option)

            newCenters = rotate(-car.wheelBase / 2, -car.width / 2, carHeadings(car_id))
            newCenters = newCenters + carCentre;
            draw_rectangle(newCenters, carHeadings(car_id), car.wheelLength, car.wheelWidth, car_wheels_color, car_wheels_color,display_option)
    else:
        newCenters = [0,0] #TODO: Should be a meaningful value

    #Draw Sensor Beams
    sensor_readings[car_id,:] = [[0 for col in range(len(sensor.angles))] for row in range(1)]
    sensor_lines[car_id] = [[0 for col in range(4)] for row in range(len(sensor.angles))]
    for i in range(1,len(sensor.angles)):
        p2 = rotate(sensor.range * math.cos(sensor.angles(i)), sensor.range * math.sin(sensor.angles(i)), carHeadings(car_id))
        p2 = p2 + carLocations[car_id,:]
        sensor_lines[car_id][i,:] = [carLocations(car_id, 1),carLocations(car_id, 2),p2(1),p2(2)]
        if (display_option == 1):
            matplotlib.line([sensor_lines[car_id][i, 1],sensor_lines[car_id][i, 3]],[sensor_lines[car_id][i, 2],sensor_lines[car_id][i, 4]], 'color', sensor_beam_color)

    # Check cars with firt draw timestep
    for i in range (1,len(prev_carLines)):
        if not prev_carLines[i]:
            prev_carLines[i] = carLines[i]

    for car_id in range( 1,len(carHeadings)):
        # Do all required intersetions for each car at once
        self_lines = [sensor_lines[car_id],carLines[car_id]]
        obstacles_lines = []
        for car2_id in range(1,len(carHeadings)):
            if (car_id != car2_id):
                obstacles_lines = [obstacles_lines, carLines[car2_id]] # current cars
        obstacles_lines = [obstacles_lines, env.lines]
        for car2_id in range( 1,len(carHeadings)):
            if (car_id != car2_id):
                obstacles_lines = [obstacles_lines, prev_carLines[car2_id]] # Step before cars
        intersections_out = matplotlib.lineSegmentIntersect(obstacles_lines, self_lines)

        # Get Sensor Reading
        for i in range (1,len(sensor.angles)):
            th = 4 * (len(carHeadings) - 1) + len(env.lines[:, 1])
            out = [intersections_out.intMatrixX[1:th, i],intersections_out.intMatrixY[1: th, i]]
            intersections = out[any[out, 2],:]

            dist2 = math.sqrt((intersections[:, 1] - carLocations[car_id, 1]) ^ 2 + (intersections[:, 2] - carLocations[car_id, 2])^ 2)
            dist_id = min(dist2)
        if not dist:
            dist = sensor.range
            if (display_option == 1 or display_option == 2):
                matplotlib.plot(sensor_lines[car_id][i, 3], sensor_lines[car_id][i, 4], 'g.')
            elif(display_option == 1 or display_option == 2):
                matplotlib.plot(intersections[id, 1], intersections[id, 2], 'g.')
            sensor_readings[car_id, i] = dist

    # Check collision
    th1 = 4 * (len(carHeadings) - 1) + 1
    th2 = len(sensor_lines[car_id][:, 1])+1
    out1 = [intersections_out.intMatrixX(th1:-1 ,th2:) ,intersections_out.intMatrixY(th1: end, th2: end)] # Car intersects a wall or another car
    intersections1 = out1[any(out1, 2),:]

    if intersections1:
        collision_bools[car_id] = 1

