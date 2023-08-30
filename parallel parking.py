import time
import random
import carla
import cv2
import glob
import os
import numpy as np
import argparse
import sys
import math 
import matplotlib as plt
#./CarlaUE4.sh -quality-level=Low
_SLEEP_TIME_ = 0.5 
STATE_SIZE = 13
invert_steering_point = -7.8
flag=0
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg'%(
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name =='nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
parked_locations = [
    carla.Transform(carla.Location(x=90.0, y=-10.0, z=0.05), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=103.0, y=-10.0, z=0.05), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=113.0, y=-10.0, z=0.05), carla.Rotation(yaw=180)),
    carla.Transform(carla.Location(x=123.0, y=-10.0, z=0.05), carla.Rotation(yaw=180))
    ]
class CarEnv():
        def __init__(self):
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(2.0)
            # Once we have a client we can retrieve the world that is currently
            # running.
            self.world = self.client.get_world()
            self.actor_list=[]
            # The world contains the list blueprints that we can use for adding new
            # actors into the simulation.
            blueprint_library = self.world.get_blueprint_library()
            # Now let's filter all the blueprints of type 'vehicle' and choose one
            # at random.
            #print(blueprint_library.filter('vehicle'))
            walker_bp = self.world.get_blueprint_library().filter("walker.pedestrian.*")

            controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
            self.model_3 = blueprint_library.filter('model3')[0]
            #self.model_3 = model_3.set_attribute('color', '255,0,0')
                    #create ego vehicle
            self.model_3_heigth = 1.443 
            self.model_3_length = 4.694 
            self.model_3_width = 2.089 
            bp = blueprint_library.filter('model3')[0]
            #bp = bp.set_attribute('color', '255,0,0')
            init_pos = carla.Transform(carla.Location(x=123.0, y=-7.0, z=0.05), carla.Rotation(yaw=180))
            self.vehicle = self.world.spawn_actor(bp, init_pos)
            self.actor_list.append(self.vehicle)
                    #create 2 parked vehicles

            for i in range(1):
                
                #location
                #start_location = carla.location(x = 115, y = -5, z = 0.05)
                trans = carla.Transform(carla.Location(x = 115, y = -5, z = 0.05), carla.Rotation(yaw=180))
                #trans.location = self.world.get_random_location_from_navigation()
                trans.location.z += 1
                
                #walker actor
                walker = random.choice(walker_bp)
                actor = self.world.spawn_actor(walker, trans)
                #self.world.wait_for_tick()

                #walker AI controller
                controller = self.world.spawn_actor(controller_bp, carla.Transform(), actor)
                #self.wait_for_tick()

                #managing pedestrian through controller
                controller.start()
                target_location = carla.Location(x=115, y=-7, z=0.05)
                controller.go_to_location(target_location)

                self.actor_list.append(actor)
                self.actor_list.append(controller)


            for pos in parked_locations:
                v = self.world.spawn_actor(bp, pos)
                self.actor_list.append(v)
            self.radar_readings = {
                                'radar_0'  : 100.0,
                                'radar_45' : 100.0,
                                'radar_90' : 100.0,
                                'radar_135': 100.0,
                                'radar_180': 100.0,
                                'radar_225': 100.0,
                                'radar_270': 100.0, 
                                'radar_315': 100.0 
                               }
            # ------------------------------ RADARS ON 8 DIFFERENT POSITION ON VEHICLE ---------------------------------- 
            radar_sensor = self.world.get_blueprint_library().find('sensor.other.radar')
            #front middle sensor
            radar_0_transform = carla.Transform(carla.Location(x=self.model_3_length/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=0.0))
            #front right sensor
            radar_45_transform = carla.Transform(carla.Location(x=self.model_3_length/2.0, y=self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=45.0))
            #right  side niddle sensor
            radar_90_transform = carla.Transform(carla.Location(x=(self.model_3_length/2.0) -1,y=self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=135.0))
            #right  side back sensor
            radar_135_transform = carla.Transform(carla.Location(x=-self.model_3_length/2.0, y=self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=135.0))
            #rear middle sensor
            radar_180_transform = carla.Transform(carla.Location(x=-self.model_3_length/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=180.0))
            #rear right sensor
            radar_225_transform = carla.Transform(carla.Location(x=-self.model_3_length/2.0, y=-self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=225.0))
            radar_270_transform = carla.Transform(carla.Location(y=-self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=270.0))
            radar_315_transform = carla.Transform(carla.Location(x=self.model_3_length/2.0, y=-self.model_3_width/2.0, z=self.model_3_heigth/2.0), carla.Rotation(yaw=315.0))

            self.radar_0 = self.world.spawn_actor(radar_sensor, radar_0_transform, attach_to=self.vehicle)
            self.radar_45 = self.world.spawn_actor(radar_sensor, radar_45_transform, attach_to=self.vehicle)
            self.radar_90 = self.world.spawn_actor(radar_sensor, radar_90_transform, attach_to=self.vehicle)
            self.radar_135 = self.world.spawn_actor(radar_sensor, radar_135_transform, attach_to=self.vehicle)
            self.radar_180 = self.world.spawn_actor(radar_sensor, radar_180_transform, attach_to=self.vehicle)
            self.radar_225 = self.world.spawn_actor(radar_sensor, radar_225_transform, attach_to=self.vehicle)
            self.radar_270 = self.world.spawn_actor(radar_sensor, radar_270_transform, attach_to=self.vehicle)
            self.radar_315 = self.world.spawn_actor(radar_sensor, radar_315_transform, attach_to=self.vehicle)

            self.actor_list.append(self.radar_0)
            self.actor_list.append(self.radar_45)
            self.actor_list.append(self.radar_90)
            self.actor_list.append(self.radar_135)
            self.actor_list.append(self.radar_180)
            self.actor_list.append(self.radar_225)
            self.actor_list.append(self.radar_270)
            self.actor_list.append(self.radar_315)

            self.radar_0.listen(lambda radar_data: self.radar_data(radar_data, key='radar_0'))
            self.radar_45.listen(lambda radar_data: self.radar_data(radar_data, key='radar_45'))
            self.radar_90.listen(lambda radar_data: self.radar_data(radar_data, key='radar_90'))
            self.radar_135.listen(lambda radar_data: self.radar_data(radar_data, key='radar_135'))
            self.radar_180.listen(lambda radar_data: self.radar_data(radar_data, key='radar_180'))
            self.radar_225.listen(lambda radar_data: self.radar_data(radar_data, key='radar_225'))
            self.radar_270.listen(lambda radar_data: self.radar_data(radar_data, key='radar_270'))
            self.radar_315.listen(lambda radar_data: self.radar_data(radar_data, key='radar_315'))

            time.sleep(_SLEEP_TIME_)
        def radar_data(self, radar_data, key):

            """
            Function for processing and storing radar readings.
                
            :params:
                - radar_data: carla.RadarMeasurement object --> array of carla.RadarDetection objects
                            containg readings from one radar sensor
                - key: key value for self.radar_readings dictionary 

            :return:
                None

            """

            radar_points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4')).reshape((len(radar_data), 4))

            if radar_points.shape[0] > 0:
                min_depth_radar_reading = min(np.reshape(radar_points[:,3],(len(radar_data),)))
            else:
                min_depth_radar_reading = 100.0

            self.radar_readings[key] = min_depth_radar_reading
        def get_current_state(self):

        # -------------------------- GETTING SENSOR READINGS ------------------------------
            current_vehicle_transform = self.vehicle.get_transform()
            current_vehicle_location = current_vehicle_transform.location
            current_vehicle_x = current_vehicle_location.x
            current_vehicle_y = current_vehicle_location.y
            angle = current_vehicle_transform.rotation.yaw

            current_vehicle_linear_velocity = self.vehicle.get_velocity().x
            current_vehicle_angular_velocity = self.vehicle.get_angular_velocity().z

            x = current_vehicle_x
            y = current_vehicle_y
            angle = self.transform_angle(angle)
            vx = current_vehicle_linear_velocity 
            wz = current_vehicle_angular_velocity  
            current_state = list(self.radar_readings.values()) + [x, y, angle, vx, wz]
                    # -------------------------- PACKING CURRENT STATE IN DICTIONARY AND ARRAY ------------------------------
            sensor_values_dict = {
                                    'x': x,
                                    'y': y,
                                    'angle': angle,
                                    'vx': vx,
                                    'wz': wz,
                                }

            current_state_dict = self.radar_readings.copy()
            current_state_dict.update(sensor_values_dict)

            current_state = np.array(current_state, dtype='float32').reshape((STATE_SIZE,))
        #    print(current_state_dict)

            return current_state, current_state_dict
        def search_spot(self):
            length_spot=0
            width_spot=0

            startTime=0
            while(1):



                self.vehicle.apply_control(carla.VehicleControl(throttle=0.3, brake=0.0))
                car_speed=abs(self.vehicle.get_velocity().x)

                cs, cd=self.get_current_state()
                print(cs[2])
                print(cs[3])
                print(cs[10])

                if(cs[2] > 1.2*self.model_3_width): 
                    startTime = time.time()  
                    depth_spot=cs[2]              #timer
                    while(cs[2] > 1.2*self.model_3_width):
                        cs,csd=self.get_current_state()

                    endTime = time.time()
                    elapsedTime = endTime - startTime
                    car_speed=abs(self.vehicle.get_velocity().x)
                    length_spot=elapsedTime*car_speed
                    print ("car speed: {} ".format(car_speed))
                    print ("length_spot: {} , depth spot: {} ".format(length_spot,depth_spot))
                if(length_spot> 1.2*self.model_3_length and depth_spot> 1.2*self.model_3_width):
                    print ("Parallel parking spot ")
                    for i in np.arange(0.3,-0.1,-0.1):
                      self.vehicle.apply_control(carla.VehicleControl(throttle=i, brake=0.0))
                      time.sleep(0.1)
                    self.parallel_park()



        def move_to_init_parking(self):
            """
            function to move the ego vehicle into 'start parking' position
            """
            while self.vehicle.get_location().x > 50:
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.4, brake=0.0))
            print("parking spot found!")
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            time.sleep(2)
        def transform_angle(self, angle):

            """
            Function for shifting angles.
                
            :params:
                - angle: float angle from range -180 deg to 180 deg

            :return:
                - angle_360: angle shifted to the range 0 deg to 360 deg

            """

            angle_360 = 360 + angle if angle < 0 else angle
            return angle_360

        def parallel_park(self):
            """
            function enables the ego vehicle to enter the parking spot also actuating on steering wheels
            todo:
            -a step function is applied on the steering wheels it would be more
            realistic to actuate the steering
            wheels with an increasing(decreasing) function i.e steer += (-)0.1
            -above mentioned approach requires a more sofisticated approach i.e. sensor based
            -last manoeuvre of the parking procedure (go forward for completing the parking)
            is based only on temporal information, variability can be caused by server delays
            """
            #while True:
                #cs,csd=self.get_current_state()
                #print ("in state 1, Right side back sensor {}".format(cs[2] ))

                #self.vehicle.apply_control(
                #    carla.VehicleControl(throttle=-0.2, steer=0.0, brake=0.0, reverse=True))
                #time.sleep(0.5)
                #print(f"angle = {cs[10]}") 
                #if cs[2]>2:
                #    break
            while True:
                cs,csd=self.get_current_state()
                print ("in state 2, Right side back sensor {}".format(cs[2] ))
                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.7, steer=0.9, brake=0.0, reverse=True))
                time.sleep(0.1)
                print(f"angle = {cs[10]}")
                if cs[10]<= 145:
                    break
            while True:
                vehicle_orientation=abs(self.vehicle.get_transform().rotation.yaw) 
                print ("in state 3, vehicle orientation {} ".format(vehicle_orientation))
                cs,csd=self.get_current_state()
                if  cs[10] >= 173.9:
                    break
                print(f"angle = {cs[10]}")
                #cs,csd=self.get_current_state()
                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.3, steer=-0.8, brake=0.0, reverse=True))
                #time.sleep(0.1)

                if  cs[10] >= 179.9:
                    break
            print("in parking line")
            self.vehicle.apply_control(
                        carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True))
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            time.sleep(0.5)
            #while True:
             #   print ("in state 4,go forward to complete the parking, front sensor {} ".format(cs[0]))
             #   cs,csd=self.get_current_state()
             #   self.vehicle.apply_control(
             #   carla.VehicleControl(throttle=0.1, steer=0.0, brake=0.0, reverse=False))
              #  if (cs[0])<1:
              #      break
            self.stop()
            return
        def stop(self):
            print("parking complete")
            while(1):
                carla.VehicleControl(throttle=0, steer=0.0, brake=1.0, reverse=False)

        def destroy(self):
            """
            destroy all the actors
            """
            print('destroying actors')
            for actor in self.actor_list:
                actor.destroy()
            print('done.')
        def run(self):
            self.search_spot()
            print("press CTRL+C to terminate the node")


def main():
    ego_vehicle = CarEnv()
 
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()            
 


    
if __name__== '__main__':
    main()
