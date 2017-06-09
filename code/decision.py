import numpy as np
from scipy.stats import mode

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.sample_detected:
            print("In sample case")
            #with open("Output.txt", "a") as text_file:
            #    print("Nav dists",np.mean(Rover.nav_dists),file=text_file)
            if (np.mean(Rover.nav_dists)<15.0 and Rover.near_sample):
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
                print("In sample case: nav case")
            else:
                if Rover.vel < Rover.max_vel-1.0:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = get_steer_angle(Rover)
            Rover.sample_detected=False
        elif Rover.mode == 'forward':
            # Check the extent of navigable terrain
            print("In forward case")
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if 0 < Rover.vel < Rover.max_vel:
                    print("In forward case:low vel")
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    Rover.steer = get_steer_angle(Rover)
                    Rover.brake=0
                elif Rover.vel <=0:
                    if not Rover.throttle == Rover.throttle_set:
                        print("In forward case:stuck")
                        Rover.throttle = Rover.throttle_set
                    else:
                        Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = np.random.randint(0,15)  # Could be more clever here about which way to turn
                else: # Else coast
                    print("In forward case:max velocity")
                    Rover.throttle = 0
                    Rover.steer = get_steer_angle(Rover)
                    Rover.brake = 0

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    print("In forward case:stuck with no terrain")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            print("In stop case")
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                print("In stop case:stop")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    print("In stop case:search nav")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = np.random.randint(0,15) # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    print("In stop case:go ahead")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = get_steer_angle(Rover)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover


def get_steer_angle(Rover):
    steer_angle = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)+np.random.randint(-5,5)
    #steer_angle = np.clip(mode(Rover.nav_angles * 180 / np.pi).mode[0]+np.random.randint(0,5), -15, 15)
    return steer_angle