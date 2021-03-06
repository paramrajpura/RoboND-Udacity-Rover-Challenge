import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function


def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        #Record start position to return back after collecting samples
        if Rover.atStart:
            Rover.start_pos = Rover.pos
            Rover.atStart = False

        #Routine to activate and reach to origin if all samples are collected
        if Rover.samples_found == 6:
            dist = np.sqrt((Rover.pos[0] - Rover.start_pos[0])**2 + (Rover.pos[1] - Rover.start_pos[1])**2)
            #with open("Output.txt", "a") as text_file:
            #    print("dist",dist,file=text_file)
            # Start directing Rover towards origin if near
            if 5 < dist <= 10:
                Rover.brake = 0
                Rover.throttle = 0
                Rover.steer = np.clip(np.arctan2(Rover.start_pos[1]- Rover.pos[1],
                                                 Rover.start_pos[0]- Rover.pos[0]),-15,15)
            # Stop if reached near origin
            elif dist <= 5:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
            # If far away, continue moving across the map
            else:
                decide_next_state(Rover)

        #Explore the map and find samples
        else:
            decide_next_state(Rover)
    return Rover


def decide_next_state(Rover):
    if Rover.sample_detected:
        #print("In sample case")
        # with open("Output.txt", "a") as text_file:
        #    print("Nav dists",np.mean(Rover.nav_dists),file=text_file)
        if Rover.near_sample:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
            #print("In sample case: nav case")
        else:
            #Control the speed to avoid a miss
            if 0 < Rover.vel < Rover.max_vel - 1.0:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            elif Rover.vel <= 0:
                if Rover.throttle == 0:
                    Rover.throttle = 2.0
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = get_steer_angle(Rover)
                else:
                    Rover.throttle = 0
                    Rover.steer = -15
                    Rover.mode = 'stop'
            else:  # Else decelerate for a while to avoid a miss
                Rover.throttle = -0.1
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            Rover.brake = 0

            #Restrict movement towards right to save time and avoid returning to same regions
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -5, 15)
        Rover.sample_detected = False
    elif Rover.mode == 'forward':
        go_forward(Rover)
    # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
    # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = -15  # -15  # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            if len(Rover.nav_angles) >= Rover.go_forward:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = get_steer_angle(Rover)
                Rover.mode = 'forward'
                #print("In stop case: data available")
        # Just to make the rover do something
        # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True


def get_steer_angle(Rover):
    # Changed clip angles to get stuck in circling loops
    # Added a bias to explore close to left wall
    steer_angle = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -10, 10)+8
    return steer_angle


def go_forward(Rover):
    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        if 0 < Rover.vel < Rover.max_vel:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
            # Set steering to average angle clipped to the range +/- 15
            Rover.steer = get_steer_angle(Rover)
        elif Rover.vel <= 0:  # Else coast
            if Rover.throttle == 0:
                Rover.throttle = 2.0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = get_steer_angle(Rover)
            else:
                Rover.throttle = 0
                Rover.steer = -15
                Rover.mode = 'stop'
        else:
            Rover.throttle = 0
            Rover.steer = get_steer_angle(Rover)
        Rover.brake = 0

    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'
