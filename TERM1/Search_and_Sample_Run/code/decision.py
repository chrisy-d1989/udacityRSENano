import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        #if every sample is collected return to start position
        if Rover.start_pos is not None:
            if np.sqrt(np.square(Rover.pos[0] - Rover.start_pos[0]) + np.square(Rover.pos[1] - Rover.start_pos[1])) < 30:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                print('returned to starting point!!!')
                return Rover
        #perform escaping of circling situation
        if Rover.mode == "circling":
            if (time.time() - Rover.start_circle_detection) < (Rover.max_circle_time + 4):
                if Rover.steer == 15:
                    print ('escaping')
                    Rover.steer = -15
                    return Rover
                else:
                    Rover.steer = 15
                    return Rover
            else:
                print('back to forward')
                Rover.start_circle_detection = None
                Rover.mode = 'forward'
                return Rover
    
        #perform escaping of stucked situation
        elif Rover.mode == 'stuck':
            if (time.time() - Rover.start_stuck_detection) < (Rover.max_stuck_time + 3):
                print('escaping')
                Rover.throttle = 0
                Rover.steer = -15
            else:
                print ('back to forward')
                Rover.start_stuck_detection = None 
                Rover.mode = 'forward'
                return Rover
    
        elif Rover.mode == 'approaching':
            #calculate distance to rock
            Rover.dist_to_rock = np.sqrt(np.square(Rover.pos[0] - Rover.rock_pos[0]) + np.square(Rover.pos[1] - Rover.rock_pos[1]))
            mean_rock_angle = np.mean(Rover.rock_angles * 180/np.pi)
            #calculate mean angle to rock
            print(Rover.dist_to_rock)
            if (time.time() - Rover.approaching_timer) < Rover.max_approaching_time:
                if Rover.vel > 1.0:
                    Rover.brake = Rover.brake_set
                elif Rover.rock_angles is not None and (Rover.dist_to_rock < 15  and Rover.dist_to_rock > 1.5) and Rover.vel <= 1.0:
                #elif Rover.vel < 1.0:    
                    if (mean_rock_angle <= 10 or mean_rock_angle >= -10):
                        Rover.brake = 0
                        Rover.throttle = Rover.approaching_throttle
                        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
                        return Rover
                    elif (mean_rock_angle < 60 or mean_rock_angle > -60) and (mean_rock_angle > 10 or mean_rock_angle < -10):
                    #else:
                        if Rover.vel > 0.1:
                            Rover.brake = Rover.brake_set
                        elif Rover.vel < 0.1:
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = mean_rock_angle / 4
                            return Rover
                #if Rover is close to sample start picking up
                elif Rover.dist_to_rock < 1.5:
                    print('rock < 1.5m')
                    # If in a state where want to pickup a rock send pickup command
                    Rover.brake = Rover.brake_set
                    Rover.throttle = 0
                    Rover.steer = 0
                    if Rover.vel == 0 and Rover.near_sample == 1:
                        #if Rover.send_pickup and not Rover.picking_up:
                        Rover.send_pickup = True
                        Rover.rock_located = False
                        Rover.approaching_timer = None
                        Rover.dist_to_rock = None
                        Rover.mode = 'forward'
                        return Rover
                #escape if sample is to far away
                elif Rover.dist_to_rock > 15.0:
                    print('to forward')
                    Rover.rock_located = False
                    Rover.approaching_timer = None
                    Rover.dist_to_rock = None
                    Rover.mode = 'forward'
                    return Rover
            #escape approaching process if not sucessfull after Rover.max_approaching_time
            if (time.time() - Rover.approaching_timer) >= Rover.max_approaching_time:
                Rover.rock_located = False
                Rover.approaching_timer = None
                Rover.dist_to_rock = None
                Rover.mode = 'forward'
                return Rover
        # Check for Rover.mode status
        elif Rover.mode == 'forward': 
            #Check if Rover is stuck
            if Rover.throttle != 0 and Rover.vel <= 0.2:
                if Rover.start_stuck_detection is None:
                    Rover.start_stuck_detection = time.time()
                elif (time.time() - Rover.start_stuck_detection) > Rover.max_stuck_time:
                    print ('Rover is stuck, start escaping')
                    Rover.mode = 'stuck'
                    return Rover
            # reset start_stuck_detection
            else:
                Rover.start_stuck_detection = None
            #Check if Rover is circling
            if (Rover.steer == -15 or Rover.steer == 15) and Rover.vel >= 1.0:
                if Rover.start_circle_detection == None:
                    Rover.start_circle_detection = time.time()
                elif (time.time() - Rover.start_circle_detection) > Rover.max_circle_time:
                    print ('Rover is circling, start escaping')
                    Rover.mode = 'circling'
                    return Rover
            else:
                Rover.start_circling_detection = None
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
  
                #increase fidelity
                if Rover.steer > 10 or Rover.steer < -10:
                    Rover.throttel = 0
                # Rover detectes rock approach it
                if Rover.rock_located is True:
                    print ('Initiating rock approaching')
                    if Rover.approaching_timer == None:
                        Rover.approaching_timer = time.time()
                    Rover.mode = 'approaching'
                    return Rover

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
                return Rover

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
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    #if there is navigable terain in front stop turning and move
                    if len(Rover.nav_angles) > Rover.go_forward:
                        Rover.steer = 0
                        Rover.mode = 'forward'
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
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

