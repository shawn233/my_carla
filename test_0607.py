import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls


import logging
from numpy import random
from queue import Queue
from queue import Empty
from matplotlib import cm
import numpy as np



VEHICLE_NUM = 10
WALKER_NUM = 8
LIDAR_NUM = 10
FRAMES = 10
CLIENT_TIMEOUT = 60.0



def spawn_vehicles(client,world, bp_lib, traffic_manager, vehicles_list):

    batch = []
    y_values = [3.75, 7.25]
    previous_x = None
    blueprints = bp_lib.filter('vehicle')

    for n in range(VEHICLE_NUM):
        if previous_x is None:
            x = random.uniform(45, 50)  
        else:
            x = previous_x + 5 + random.uniform(0, 5)

        y = random.choice(y_values)

        transform = carla.Transform(carla.Location(x=x, y=y, z=0.01))

            
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # spawn the cars and set their autopilot and light state all together
        batch.append(carla.command.SpawnActor(blueprint, transform)
                .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))
            
        previous_x = x

    responses = client.apply_batch_sync(batch, True)
            
    for response in responses:
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
            vehicle = world.get_actor(response.actor_id)
            print('created %s, x=%.2f, y=%.2f' % (vehicle.type_id, vehicle.get_location().x, vehicle.get_location().y))

    return vehicles_list

def spawn_walkers(client, world, bp_lib, vehicles_list, walkers_list, all_id):
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road

    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(WALKER_NUM):
        spawn_point = carla.Transform()
        x = random.uniform(50, 150)
        y = random.uniform(12,13) # TODO 人的出生点Y值在车道上
        loc = carla.Location(x=x, y=y, z=0)
        
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
            print(f'walker spawn point x={spawn_point.location.x:.2f} y={spawn_point.location.y:.2f}')

    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        blueprints = bp_lib.filter('walker')
        walker_bp = random.choice(blueprints)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        print('spawn walker has error', results[i].has_error())
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    print(walkers_list)

    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id

    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        loc = world.get_random_location_from_navigation()
        print(i, all_actors[i], loc)
        all_actors[i].go_to_location(loc)
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

    return all_actors

def spawn_Lidars(world,vehicles_list,sensor_list):


    if not os.path.isdir('_out'):
        os.mkdir('_out')
    # Search the desired blueprints
    bp_lib = world.get_blueprint_library()
    lidar_bp = bp_lib.filter("sensor.lidar.ray_cast")[0]

        

    # Configure the blueprints
    lidar_bp.set_attribute('dropoff_general_rate', '0.0')
    lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
    lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
    lidar_bp.set_attribute('horizontal_fov', '360.0')
    lidar_bp.set_attribute('rotation_frequency','20.0') 
    lidar_bp.set_attribute('upper_fov', '30.0')
    lidar_bp.set_attribute('lower_fov', '-25.0')
    lidar_bp.set_attribute('channels', '64.0')
    lidar_bp.set_attribute('range', '100.0')
    lidar_bp.set_attribute('points_per_second', '100000')
        

    # Spawn the blueprints
    lidar_location = carla.Location(0, 0, 2)
    lidar_rotation = carla.Rotation(0, 0, 0)
    
    lidar = world.spawn_actor(
        blueprint=lidar_bp,
        transform=carla.Transform(lidar_location, lidar_rotation),
        attach_to=world.get_actor(vehicles_list[0]))
        

    # The sensor data will be saved in thread-safe Queues
    lidar_queue = Queue()

    lidar.listen(lambda data: lidar_queue.put(data))

    for frame in range(FRAMES):
        world.tick()
        world_frame = world.get_snapshot().frame

        try:
            # Get the data once it's received.
            lidar_data = lidar_queue.get(True, 1.0)
        except Empty:
            print("[Warning] Some sensor data has been missed")
            continue

        assert lidar_data.frame == world_frame
        # At this point, we have the synchronized information from the sensor.
        sys.stdout.write("\r(%d/%d) Simulation: %d Lidar: %d" %
            (frame, FRAMES, world_frame, lidar_data.frame) + ' ')
        sys.stdout.flush()

        # Get the lidar data and convert it to a numpy array.
        p_cloud_size = len(lidar_data)
        p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
        p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

        # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
        # focus on the 3D points.
        intensity = np.array(p_cloud[:, 3])

        # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
        local_lidar_points = np.array(p_cloud[:, :3]).T
            

        # Add an extra 1.0 at the end of each 3d point so it becomes of
        # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
        local_lidar_points = np.r_[
            local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

        # This (4, 4) matrix transforms the points from lidar space to world space.
        lidar_2_world = lidar.get_transform().get_matrix()

        # Transform the points from lidar space to world space.
        world_points = np.dot(lidar_2_world, local_lidar_points)
        

        # Save the point cloud data to a file
        np.savetxt("_out/%08d.txt" % lidar_data.frame, world_points.T, fmt='%.4f')
    sensor_list.append(lidar)
    return sensor_list 

            




def main():
    random.seed(666)
    np.random.seed(6667)

    vehicles_list = []
    walkers_list = []
    sensor_list = []
    all_id = []
    all_actors = None

    # Connect to the server
    client = carla.Client('localhost', 2000)
    client.set_timeout(CLIENT_TIMEOUT)
    world = client.load_world('Town03')
    # if world is None:
    #     world = client.get_world()
    print('world', world)
    bp_lib = world.get_blueprint_library()

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = None #0.05
    world.apply_settings(settings)

    try:
        vehicles_list = spawn_vehicles(client, world, bp_lib, traffic_manager, vehicles_list)
        all_actors = spawn_walkers(client,world, bp_lib, vehicles_list,walkers_list,all_id)
        sensor_list = spawn_Lidars(world,vehicles_list,sensor_list)

        while True:
            # Update world state
            world.tick()  # Or use world.wait_for_tick()

            # Perform other operations such as handling sensors, controlling vehicles, etc.
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been caught.")



    finally:
        world.apply_settings(original_settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
            
        #     all_actors[i].stop()
        #     all_actors[i+1].destroy()

        print(all_actors)

        for idx, a in enumerate(all_actors):
            if idx % 2 == 0:
                a.stop()
            a.destroy()

        # Destroy the lidar in the scene.
        for sensor in sensor_list:
            sensor.destroy()


        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')


# TODO 行人未清理
# TODO  Simulation: 79125 Lidar: 79125 日志里数字的含义