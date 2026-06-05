import coppeliasim_zmqremoteapi_client as zmqRemoteApi
import onnxruntime as ort
import numpy as np
from scipy.spatial.transform import Rotation
from Quaternion import Quaternion
import random
from itertools import product
import os

table_z = -0.3534

def create_homogeneous_transformation(T_3x3):
    T = np.eye(4)
    T[:3, :3] = T_3x3
    return T

def to_one_hot(n, max):
    one_hot_vector = [0 for _ in range(max)]
    one_hot_vector[n] = 1
    return one_hot_vector

def consistent_quaternion(q):
    return q if q[3] >= 0 else [-q[0], -q[1], -q[2], -q[3]]


class UnityAgent:
    def __init__(self, grasp_and_lift_path, place_path):
        self.grasp_and_lift_policy = ort.InferenceSession(grasp_and_lift_path)
        self.place_policy = ort.InferenceSession(place_path)

    def predict(self, obs, phase):
        model = None
        if phase == 0:
            model = self.grasp_and_lift_policy
        elif phase == 2:
            model = self.place_policy
        
        if model is not None:
            prediction = model.run(None, {"obs_0": np.expand_dims(obs, 0).astype(np.float32), 
                                        "action_masks": np.ones((1, 2)).astype(np.float32)})
            return prediction[2][0][:3], prediction[2][0][3:7], int(prediction[5][0][0])        # Displacement, delta rotation, open-close discrete action
        else:
            return None

class CoppeliaEnv:
    def __init__(self, max_magnitude = 0.005, rot_limit = 1, max_speed = 0.03, angular_speed = 0.03):
        self.max_magnitude = max_magnitude
        self.rot_limit = rot_limit
        self.speed = max_speed
        self.angular_speed = angular_speed
        self.T = create_homogeneous_transformation(np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]]))    # Coordinate frame transform: Unity (left-handed, Y-up) → CoppeliaSim (right-handed, Z-up)
        self.unity_fixed_delta_time = 0.01
        self.min_y = -0.2
        self.max_y = 0.2
        self.min_x = 0.125
        self.max_x = 0.351
        self.max_x_obj_clean_and_place = 0.15
        self.min_x_box_clean_and_place = 0.25
        self.min_scale_x_y = 0.03
        self.max_scale_x_y = 0.04
        self.min_scale_z = 0.03
        self.max_scale_z = 0.14
        self.min_scale_cuboid = 0.035
        self.max_scale_cuboid = 0.04
        self.min_mass = 0.5
        self.max_mass = 1
        self.default_rot_top = [0, 0, 0, 1]
        self.default_rot_side = [-0.2705980500731013, -0.2705980500731292, 0.6532814824382148, 0.6532814824381478]
        self.objects = []
        self.obj_starting_z = None
        self.lift_offset = 0.026  # 0.2 for pick-and-place, 0.026 for grasp-and-lift
        self.spawn_area_height = None
        self.spawn_area_width = None
        self.obstacles_grid_coordinates = None
        self.obj_grid_coordinates = None
        self.obstacles_grid_coordinates_indexes = None
        self.obj_grid_coordinates_indexes = None
        self.grid_width = 0
        self.grid_height = 0
        self.obj_grid_dict = {}

    def randomize_obj_and_target_pos(self):
        self.sim.scaleObject(self.obj_handle, )

    def check_fallen_object(self):
        return self.sim.getObjectPosition(self.obj_handle, self.reference_frame)[2] < -0.5

    def start_simulation(self):
        self.sim.startSimulation()
        while self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.switchThread()
        self.sim.wait(20)
    
    def stop_simulation(self):
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            self.sim.switchThread()

    def simulate_unity_movement(self):
        unity_movement = (np.array(self.sim.getObjectPosition(self.target_handle)) - np.array(self.sim.getObjectPosition(self.ik_target))) * self.speed
        rb_position = self.sim.getObjectPosition(self.ik_target) + unity_movement
        self.sim.setObjectPosition(self.ik_target, rb_position.tolist())

    def move_target_with_displacement(self, displacement):
        self.simulate_unity_movement()
        current_position = np.array(self.sim.getObjectPosition(self.target_handle, self.sim.handle_parent))
        displacement = np.array(displacement)
        target_position = current_position + displacement

        self.sim.setObjectPosition(self.target_handle, target_position.tolist(), self.sim.handle_parent)

    def apply_local_translation(self, displacement):
        displacement = self.pos_unity_to_coppelia(displacement)
        if np.linalg.norm(displacement) > self.max_magnitude:
            displacement = displacement / np.linalg.norm(displacement) * self.max_magnitude
        self.move_target_with_displacement( displacement)

    def delta_rot(self, rotation, delta_rotation):
        if np.linalg.norm(delta_rotation - np.array([0, 0, 0, 1])) <= 1e-4:
            return rotation
        if not isinstance(rotation, Quaternion):
            rotation = Quaternion(*rotation)
        delta_rotation = Quaternion(*delta_rotation)
        angle, axis = delta_rotation.ToAngleAxis()
        angle = min(angle, self.rot_limit)
        limited_rotation = Quaternion.AngleAxis(angle, axis)
        return rotation * limited_rotation

    def apply_delta_rotation(self, delta_rotation):
        IK_target_rot = Rotation.from_quat(self.sim.getObjectQuaternion(self.ik_target))
        rotation = Rotation.from_quat(self.sim.getObjectQuaternion(self.target_handle))
        rotation_difference = IK_target_rot.inv() * rotation
        angle_rad, axis = rotation_difference.magnitude(), rotation_difference.as_rotvec()
        if np.linalg.norm(axis) > 0:
            axis = axis / np.linalg.norm(axis)
            limited_rot = Rotation.from_rotvec(angle_rad * axis * self.angular_speed)
        else:
            limited_rot = Rotation.identity()
        unity_movement = IK_target_rot * limited_rot
        self.sim.setObjectQuaternion(self.ik_target, consistent_quaternion(unity_movement.as_quat()))

        unity_rot, _ = self.coppelia_to_unity(Rotation.from_quat(self.sim.getObjectQuaternion(self.target_handle)).as_matrix(), [0, 0, 0])
        rotation = Quaternion(*unity_rot)

        new_rot = self.delta_rot(rotation, delta_rotation)
        new_rot = [new_rot.x, new_rot.y, new_rot.z, new_rot.w]

        quat_coppelia, _ = self.unity_to_coppelia(Rotation.from_quat(new_rot).as_matrix(), [0, 0, 0])
        self.sim.setObjectQuaternion(self.target_handle, quat_coppelia)

    def obj_target_collision(self):
        return self.sim.checkCollision(self.obj_handle, self.target_zone)[0] == 1

    def obj_landing_zone_collision(self):
        return self.sim.checkCollision(self.obj_handle, self.land_zone)[0] == 1

    def start_reach_target_phase(self):
        return self.sim.getObjectPosition(self.obj_handle, self.reference_frame)[2] >= self.obj_starting_z + self.lift_offset

    def check_obj_away(self):
        return np.linalg.norm(np.array(self.sim.getObjectPosition(self.obj_handle, self.reference_frame)) - np.array(self.sim.getObjectPosition(self.end_effector, self.reference_frame))) > 0.15

    def step(self, displacement, delta_rotation, close_gripper):
        self.apply_delta_rotation(delta_rotation)
        self.apply_local_translation(displacement)
        self.sim.setInt32Signal('closing', close_gripper)
    
    def step_simulation(self):
        self.sim.step()
            
    def open_gripper(self):
        self.sim.setInt32Signal('closing', 0)
    
    # https://robotics.stackexchange.com/questions/114524/how-to-convert-rotations-between-unity-left-handed-y-up-and-coppeliasim-righ
    def coppelia_to_unity(self, coppelia_R, coppelia_t):
        H = np.eye(4)
        H[:3, :3] = coppelia_R
        H[:3, 3] = coppelia_t
        H_unity = np.linalg.inv(self.T) @ H @ self.T
        return Rotation.from_matrix(H_unity[:3, :3]).as_quat(), H_unity[:3, 3]

    def unity_to_coppelia(self, unity_R, unity_t):
        H = np.eye(4)
        H[:3, :3] = unity_R
        H[:3, 3] = unity_t
        H_coppelia = self.T @ H @ np.linalg.inv(self.T)
        return Rotation.from_matrix(H_coppelia[:3, :3]).as_quat(), H_coppelia[:3, 3]
    
    def pos_unity_to_coppelia(self, pos_unity):
        _, pos_coppelia = self.unity_to_coppelia(np.eye(3), pos_unity)
        return pos_coppelia

    def pos_coppelia_to_unity(self, pos_coppelia):
        _, pos_unity = self.coppelia_to_unity(np.eye(3), pos_coppelia)
        return pos_unity

    def eulers_coppelia_to_unity(self, eulers_coppelia):
        quat = Rotation.from_euler('xyz', eulers_coppelia, degrees=False).as_quat()
        unity_rot, _ = self.coppelia_to_unity(Rotation.from_quat(quat).as_matrix(), [0, 0, 0])
        return Rotation.from_quat(unity_rot).as_euler('xyz', degrees=False)

    def grabbed(self):
        is_finger1_touching, _ = self.sim.checkCollision(self.left_finger, self.obj_handle)
        is_finger2_touching, _ = self.sim.checkCollision(self.right_finger, self.obj_handle)
        return is_finger1_touching and is_finger2_touching

    def get_object_local_pos(self):
        return self.sim.getObjectPosition(self.obj_handle, self.reference_frame)

    def get_obj_bounding_box_points(self):
        size, _ = self.sim.getShapeBB(self.obj_handle)
        half_x, half_y, half_z = size[0] / 2, size[1] / 2, size[2] / 2

        local_points = [
            (-half_x, half_y, half_z),   # FrontTopLeft
            (-half_x, -half_y, half_z),    # FrontTopRight
            (-half_x, half_y, -half_z),  # FrontBottomLeft
            (-half_x, -half_y, -half_z),   # FrontBottomRight
            (half_x, half_y, half_z),  # BackTopLeft
            (half_x, -half_y, half_z),   # BackTopRight
            (half_x, half_y, -half_z), # BackBottomLeft
            (half_x, -half_y, -half_z)   # BackBottomRight
        ]

        parent_matrix = self.sim.getObjectMatrix(self.obj_handle, -1)

        # Local to global
        global_points = [self.sim.multiplyVector(parent_matrix, point) for point in local_points]

        return global_points

    def global_to_local(self, global_point):
        obj_matrix = self.sim.getObjectMatrix(self.reference_frame, -1)
        matrix = np.array(obj_matrix).reshape(3, 4)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = matrix[:, :3]  # Rotation
        transformation_matrix[:3, 3] = matrix[:, 3]  # Traslation
        matrix_inv = np.linalg.inv(transformation_matrix)

        p_homogeneous = np.array([global_point[0], global_point[1], global_point[2], 1])
        
        # Global to local
        p_local = matrix_inv @ p_homogeneous
        
        return p_local[:3]

    def get_hand_angular_velocity(self):
        target_rotation = Rotation.from_quat(self.sim.getObjectQuaternion(self.target_handle))
        IK_target_rot = Rotation.from_quat(self.sim.getObjectQuaternion(self.ik_target))
        rotation_difference = target_rotation * IK_target_rot.inv()
        angle_rad, axis = rotation_difference.magnitude(), rotation_difference.as_rotvec()
        new_rot = self.angular_speed * angle_rad * axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 0 else np.array([0, 0, 0])
        angular_velocity = Rotation.from_rotvec(new_rot / 0.01)
        return angular_velocity.as_euler('xyz')

    def get_hand_linear_velocity(self):
        return (np.array(self.sim.getObjectPosition(self.target_handle)) - np.array(self.sim.getObjectPosition(self.ik_target))) * self.speed / self.unity_fixed_delta_time

    def set_obj_handle(self, obj):
        self.obj_handle = obj
        self.obj_starting_z = self.sim.getObjectPosition(self.obj_handle, self.reference_frame)[2]

    def get_observations_grasp_and_lift(self, task):
        obs = []
        bb_points = self.get_obj_bounding_box_points()
        end_effector_pos = np.array(self.sim.getObjectPosition(self.end_effector))
        obj_pos = self.sim.getObjectPosition(self.obj_handle, self.reference_frame)
        obj_pos = self.pos_coppelia_to_unity(obj_pos)
        obj_rot, _ = self.coppelia_to_unity(Rotation.from_quat(self.sim.getObjectQuaternion(self.obj_handle)).as_matrix(), [0, 0, 0])
        hand_base_rot, _ = self.coppelia_to_unity(Rotation.from_quat(self.sim.getObjectQuaternion(self.ik_target)).as_matrix(), [0, 0, 0])

        for point in bb_points:
            diff = self.pos_coppelia_to_unity(np.array(point) - end_effector_pos)
            obs.extend(diff)

        obs.extend(obj_pos)
        obs.extend(obj_rot)

        obj_linear_velocity, obj_angular_velocity = self.sim.getObjectVelocity(self.obj_handle)

        obs.extend(self.pos_coppelia_to_unity(obj_linear_velocity))
        obs.extend(self.eulers_coppelia_to_unity(obj_angular_velocity))

        obs.extend(np.array(self.pos_coppelia_to_unity(self.sim.getObjectPosition(self.end_effector, self.reference_frame))))
        obs.extend(hand_base_rot)
        obs.extend(self.pos_coppelia_to_unity(self.get_hand_linear_velocity()))
        
        angular_velocity = self.get_hand_angular_velocity()
        obs.extend(self.eulers_coppelia_to_unity(angular_velocity))
        obs.append(self.grabbed())
        obs.extend(to_one_hot(task, 2))
        return np.array(obs)

    def get_observations_place(self):
        obs = []
        bb_points = self.get_obj_bounding_box_points()
        obj_pos = self.sim.getObjectPosition(self.obj_handle, self.reference_frame)
        obj_pos = self.pos_coppelia_to_unity(obj_pos)
        obj_rot, _ = self.coppelia_to_unity(Rotation.from_quat(self.sim.getObjectQuaternion(self.obj_handle)).as_matrix(), [0, 0, 0])
        hand_base_rot, _ = self.coppelia_to_unity(Rotation.from_quat(self.sim.getObjectQuaternion(self.ik_target)).as_matrix(), [0, 0, 0])

        for point in bb_points:
            obs.extend(self.pos_coppelia_to_unity(self.global_to_local(point)))
        
        target_pos = np.array(self.sim.getObjectPosition(self.object_target))
        front_bottom_left_dist = self.pos_coppelia_to_unity(target_pos - bb_points[2])
        front_bottom_right_dist = self.pos_coppelia_to_unity(target_pos - bb_points[3])
        back_bottom_left_dist = self.pos_coppelia_to_unity(target_pos - bb_points[6])
        back_bottom_right_dist = self.pos_coppelia_to_unity(target_pos - bb_points[7])

        obs.extend((front_bottom_left_dist + front_bottom_right_dist + back_bottom_left_dist + back_bottom_right_dist) / 4)

        obs.extend(obj_rot)

        obj_linear_velocity, obj_angular_velocity = self.sim.getObjectVelocity(self.obj_handle)

        obs.extend(self.pos_coppelia_to_unity(obj_linear_velocity))
        obs.extend(self.eulers_coppelia_to_unity(obj_angular_velocity))

        obs.extend(self.pos_coppelia_to_unity(self.sim.getObjectPosition(self.object_target, self.reference_frame)))

        obs.extend(np.array(self.pos_coppelia_to_unity(self.sim.getObjectPosition(self.end_effector, self.reference_frame))))
        obs.extend(hand_base_rot)
        obs.extend(self.pos_coppelia_to_unity(self.get_hand_linear_velocity()))
        
        angular_velocity = self.get_hand_angular_velocity()
        obs.extend(self.eulers_coppelia_to_unity(angular_velocity))
        obs.append(self.grabbed())
        return np.array(obs)

    def remove_objs(self):
        self.sim.removeObjects(self.objects)
        self.objects = []

    def remove_obj(self):
        self.sim.removeObjects([self.obj_handle])

    def dist(self, pos1, pos2):
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def check_dist(self, pos, rot, max_dist=0.005, max_angular_dist=0.005):
        rot_check = True if rot is None else self.angular_distance(self.sim.getObjectQuaternion(self.ik_target), rot) <= max_angular_dist
        return self.dist(self.sim.getObjectPosition(self.ik_target, self.reference_frame), pos) <= max_dist and rot_check

    def angular_distance(self, q1, q2):
        q1 = Rotation.from_quat(q1)
        q2 = Rotation.from_quat(q2)
        delta_rotation = q1.inv() * q2
        if np.linalg.norm(np.array(delta_rotation.as_quat()) - np.array([0, 0, 0, 1])) <= 1e-4:
            return 0
        consistent_rot = Rotation.from_quat(consistent_quaternion(delta_rotation.as_quat()))
        angle = np.rad2deg(consistent_rot.magnitude())
        return angle

    def move_target(self, pos, rot=None):
        displacement = np.array(pos) - np.array(self.sim.getObjectPosition(self.target_handle, self.reference_frame))
        delta_rotation = [0, 0, 0, 1]
        if rot is not None:
            target_rot = Rotation.from_quat(rot)
            current_rot = Rotation.from_quat(self.sim.getObjectQuaternion(self.target_handle))
            delta_rotation = current_rot.inv() * target_rot
            delta_rotation, _ = self.coppelia_to_unity(delta_rotation.as_matrix(), [0, 0, 0])
            delta_rotation = consistent_quaternion(delta_rotation)
        return self.pos_coppelia_to_unity(displacement), delta_rotation

    def check_objects_collision_2d(self, pos, dist):
        for obj in self.objects:
            if np.linalg.norm(np.array(self.sim.getObjectPosition(obj, self.reference_frame)[:2]) - np.array(pos)) < dist:    
                return True
        return False
    
    def check_collision_2d(self, pos1, pos2, dist):
        return np.linalg.norm(np.array(pos1) - np.array(pos2)) < dist

    def load_stl(self, filename, scaling_factor=0.001):
        shape_handle = self.sim.importShape(0, filename, 32, 0, scaling_factor)
        self.sim.setObjectInt32Param(shape_handle, self.sim.shapeintparam_culling, 0)  # To avoid rendering problems
        inertia, com = self.sim.getShapeInertia(shape_handle)
        size, _ = self.sim.getShapeBB(shape_handle)
        com[11] = size[2] / 2
        self.sim.setShapeInertia(shape_handle, inertia, com)
        return shape_handle

    def spawn_random_object(self, clean_and_place=False, obj=0):
        trials = 0
        obj_handle = None
        while True:
            if obj == 0:
                scale_x_y = random.uniform(self.min_scale_x_y, self.max_scale_x_y)
                obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [1, 1, 1])
                scale_z = random.uniform(self.min_scale_z, self.max_scale_z)
            elif obj == 1:
                scale_x_y = scale_z = random.uniform(self.min_scale_cuboid, self.max_scale_cuboid)
                obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cuboid, [1, 1, 1])
                self.sim.setShapeColor(obj_handle, None, self.sim.colorcomponent_ambient_diffuse, [1, 0.8, 0])
            elif obj == 2:  # Bottle
                scale_x_y = random.uniform(0.04, 0.05) / 0.06022
                scale_z = 1
                obj_handle = self.load_stl(os.getcwd() + "\objects\water_bottle.stl", 0.001)
            elif obj == 3:  # Wine glass
                rand_value = random.uniform(0.8, 1)
                scale_x_y = rand_value
                scale_z = rand_value
                obj_handle = self.load_stl(os.getcwd() + "\objects\wine_glass.stl", 0.0008)
                self.sim.setShapeColor(obj_handle, None, self.sim.colorcomponent_transparency, [0.45])
            elif obj == 4:  # Cup
                scale_x_y = 0.8
                scale_z = 0.8
                obj_handle = self.load_stl(os.getcwd() + "\objects\cup.stl", 0.0008)
            elif obj == 5:  # Mug
                scale_x_y = 1.05
                scale_z = 1.05
                obj_handle = self.load_stl(os.getcwd() + "\objects\mug.stl", 0.0008)

            self.sim.scaleObject(obj_handle, scale_x_y, scale_x_y, scale_z)
            size, _ = self.sim.getShapeBB(obj_handle)

            if obj >= 2:
                obj_pos = self.sim.getObjectPosition(obj_handle)
                new_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + size[2] / 2]
                self.sim.relocateShapeFrame(obj_handle, [*new_pos, *self.sim.getObjectQuaternion(obj_handle)])

            if clean_and_place:
                x = random.uniform(self.min_x, self.max_x_obj_clean_and_place)
            else:
                x = random.uniform(self.min_x, self.max_x)
            
            z = table_z + (size[2] / 2)
            pos = [x, random.uniform(self.min_y, self.max_y), z]
            if self.check_objects_collision_2d(pos[:2], 0.1) or self.check_collision_2d(pos[:2], self.sim.getObjectPosition(self.object_target, self.reference_frame)[:2], 0.15):   # max_scale x_y is 0.04, so two max-scale objects need >= 0.08 clearance; 0.1 adds margin. Same logic for collision with the box target.
                self.sim.removeObjects([obj_handle])
                trials += 1
                if trials > 1000:
                    print("Spawn error")
                    return None
            else:
                self.sim.setObjectPosition(obj_handle, pos, self.reference_frame)
                self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.setObjectFloatParam(obj_handle, self.sim.shapefloatparam_mass, random.uniform(self.min_mass, self.max_mass))
                self.sim.setFloatProperty(obj_handle, "bullet.friction", 0.6)
                self.sim.setProperty(obj_handle, 'collidable', True)
                break
        return obj_handle
    
    def spawn_default_cylinder(self):
        obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [1, 1, 1])
        self.sim.scaleObject(obj_handle, 0.035, 0.035, 0.13)
        size, _ = self.sim.getShapeBB(obj_handle)
        self.sim.setObjectPosition(obj_handle, [0.250, 0, table_z + (size[2] / 2)], self.reference_frame)
        return obj_handle

    def place_target_pos_default(self):
        self.sim.setObjectPosition(self.box, [0.250, -0.22665, -0.325], self.reference_frame)

    def calculate_half_area_grid(self):
        """Compute the discrete grid over the half-area used for obstacle/object placement.

        Coordinates are sorted by x (front-to-back) with randomised y order so that
        objects placed at the front of the table appear first in the grasp queue.
        """
        self.spawn_area_width = self.max_y - self.min_y
        self.spawn_area_height = self.max_x - self.min_x
        self.grid_width = (self.spawn_area_width / 2) // self.max_scale_x_y
        self.grid_height = self.spawn_area_height // self.max_scale_x_y
        prod = list(product(range(int(self.grid_height)), range(int(self.grid_width))))

        # Sort by x (front first) but randomise y so obstacles are spread laterally.
        # Objects at the front of the table are encountered first during grasping.
        dict = {index: coord for index, coord in enumerate(prod) if coord[0] < int(self.grid_height) - 2}
        items = list(dict.values())
        random.shuffle(items)
        dict_y_sorted = sorted(items, key=lambda item: item[0], reverse=False)
        self.obstacles_grid_coordinates = {i: coord for i, coord in enumerate(dict_y_sorted)}

        self.obstacles_grid_coordinates_indexes = {self.obstacles_grid_coordinates[index]: index for index in self.obstacles_grid_coordinates}
        self.bottle_min_x = self.min_x + (int(self.grid_height) - 1) * self.max_scale_x_y + (self.max_scale_x_y / 2)


    def spawn_random_object_half_area_grid(self, half, obj=0, obstacle=True):
        """Spawn an object on the grid within the robot's half of the table.

        Obstacles are placed at precomputed chessboard-like grid positions; the
        target object is placed randomly in the far rows reserved for it.
        """
        obj_handle = None
        starting_y = self.min_y if half == 1 else self.min_y + (self.spawn_area_width / 2)
        movable = 0     # Distinguishes immovable obstacles from movable objects

        grid_len_x = int((self.max_x - self.min_x) / self.max_scale_x_y)
        grid_len_y = int((self.spawn_area_width / 2) / self.max_scale_x_y)

        # Maximum index = length - 1
        max_idx_x = grid_len_x - 1
        max_idx_y = grid_len_y - 1
        
        if obstacle:
            coord_dict = self.obstacles_grid_coordinates if obstacle else self.obj_grid_coordinates
            inv_coord_dict = self.obstacles_grid_coordinates_indexes if obstacle else self.obj_grid_coordinates_indexes
            index = 0
            random_coord = coord_dict.pop(list(coord_dict.keys())[int(index)], None)
            # Now we remove the coordinates in the neighborhood (not diagonal) in order to get a chessboard-like positioning
            coord_dict.pop(inv_coord_dict.pop((random_coord[0] + 1, random_coord[1]), None), None)
            coord_dict.pop(inv_coord_dict.pop((random_coord[0] - 1, random_coord[1]), None), None)
            coord_dict.pop(inv_coord_dict.pop((random_coord[0], random_coord[1] + 1), None), None)
            coord_dict.pop(inv_coord_dict.pop((random_coord[0], random_coord[1] - 1), None), None)
            table_coord = [self.min_x + random_coord[0] * self.max_scale_x_y + (self.max_scale_x_y / 2), starting_y + random_coord[1] * self.max_scale_x_y + (self.max_scale_x_y / 2)]
            
        else:
            y = starting_y + random.uniform(0, (self.spawn_area_width / 2))
            table_coord = [random.uniform(self.bottle_min_x, self.max_x), y]

            idx_x = int((table_coord[0] - self.min_x) / self.max_scale_x_y)
            idx_y = int((table_coord[1] - starting_y) / self.max_scale_x_y)
            
            random_coord = (idx_x, idx_y)

        if obj == 0: # Cylinder
            scale_x_y = random.uniform(self.min_scale_x_y, self.max_scale_x_y)
            obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [1, 1, 1])
            scale_z = random.uniform(self.min_scale_z, self.max_scale_z)
            self.sim.setShapeColor(obj_handle, None, self.sim.colorcomponent_ambient_diffuse, [1, 0.36, 0.36])
            movable = 1
        elif obj == 1: # Cuboid
            scale_x_y = scale_z = random.uniform(self.min_scale_cuboid, self.max_scale_cuboid)
            obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cuboid, [1, 1, 1])
            movable = 1
        elif obj == 2:  # Bottle
            scale_x_y = 0.04 / 0.06022
            scale_z = 1
            obj_handle = self.load_stl(os.getcwd() + "\objects\water_bottle.stl", 0.001)
            self.sim.setShapeColor(obj_handle, None, self.sim.colorcomponent_ambient_diffuse, [0.24, 0.80, 1])
            self.sim.setShapeColor(obj_handle, None, self.sim.colorcomponent_transparency, [0.80])
            movable = 1
        elif obj == 3:  # Wine glass
            scale_x_y = 1
            scale_z = 1
            obj_handle = self.load_stl(os.getcwd() + "\objects\wine_glass.stl", 0.001)
            movable = 1

        self.sim.scaleObject(obj_handle, scale_x_y, scale_x_y, scale_z)
        size, _ = self.sim.getShapeBB(obj_handle)
        if obj >= 2:
            obj_pos = self.sim.getObjectPosition(obj_handle)
            new_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + size[2] / 2]
            self.sim.relocateShapeFrame(obj_handle, [*new_pos, *self.sim.getObjectQuaternion(obj_handle)])
        
        table_coord.append(table_z + (size[2] / 2))

        self.sim.setObjectPosition(obj_handle, table_coord, self.reference_frame)
        self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_static, 0)
        self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_respondable, 1)
        self.sim.setObjectFloatParam(obj_handle, self.sim.shapefloatparam_mass, random.uniform(self.min_mass, self.max_mass))
        self.sim.setFloatProperty(obj_handle, "bullet.friction", 0.6)
        
        # Mirror coordinates so the planning grid matches the robot's perspective:
        # Horizontal flip: (max - x)
        dict_x = max_idx_x - random_coord[0]
        # Vertical flip: (max - y)
        dict_y = max_idx_y - random_coord[1]
        dict_x = max(0, dict_x)
        dict_y = max(0, dict_y)
        planning_grid_coord = (dict_x, dict_y)
        self.obj_grid_dict[planning_grid_coord] = (obj_handle, obj, movable)
        
        return obj_handle
    

    def spawn_random_object_half_area(self, half, obj=0):
        trials = 0
        obj_handle = None
        spawn_area_width = self.max_y - self.min_y
        while True:
            if obj == 0: # Cylinder
                scale_x_y = random.uniform(self.min_scale_x_y, self.max_scale_x_y)
                obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, [1, 1, 1])
                scale_z = random.uniform(self.min_scale_z, self.max_scale_z)
            elif obj == 1: # Cuboid
                scale_x_y = scale_z = random.uniform(self.min_scale_cuboid, self.max_scale_cuboid)
                obj_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cuboid, [1, 1, 1])
            elif obj == 2:  # Bottle
                scale_x_y = 0.04 / 0.06022
                scale_z = 1
                obj_handle = self.load_stl(os.getcwd() + "\objects\water_bottle.stl", 0.001)
                
                vertices, indices = self.sim.getShapeMesh(obj_handle)
                convex_hull_handle = self.sim.createShape(2, 0, vertices, indices)
                convex_pieces = self.sim.convexDecompose(obj_handle, 0, [0, 16, 1], [1.0, 0.0]) # Concavity alta
                bottle_hull = convex_pieces[0] # Dovrebbe essercene solo uno
                self.sim.removeObjects([obj_handle])
                obj_handle = convex_hull_handle
            
            self.sim.scaleObject(obj_handle, scale_x_y, scale_x_y, scale_z)
            size, _ = self.sim.getShapeBB(obj_handle)

            if obj == 2:
                obj_pos = self.sim.getObjectPosition(obj_handle)
                new_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + size[2] / 2]
                self.sim.relocateShapeFrame(obj_handle, [*new_pos, *self.sim.getObjectQuaternion(obj_handle)])

            x = random.uniform(self.min_x, self.max_x)
            limit = (size[1] / 2) + 0.01
            if half == 0:
                y_coord = self.min_y + (spawn_area_width / 2) + random.uniform(limit, spawn_area_width / 2)
            else:
                y_coord = self.min_y + random.uniform(0, (spawn_area_width / 2) - limit)
            pos = [x, y_coord, table_z + (size[2] / 2)]
            if self.check_objects_collision_2d(pos[:2], 0.05) or self.check_collision_2d(pos[:2], self.sim.getObjectPosition(self.object_target, self.reference_frame)[:2], 0.15):   # max_scale x_y is 0.04, so two max-scale objects need >= 0.08 clearance; 0.1 adds margin. Same logic for collision with the box target.
                self.sim.removeObjects([obj_handle])
                trials += 1
                if trials > 1000:
                    print("Spawn error")
                    return None
            else:
                self.sim.setObjectPosition(obj_handle, pos, self.reference_frame)
                self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(obj_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.setObjectFloatParam(obj_handle, self.sim.shapefloatparam_mass, random.uniform(self.min_mass, self.max_mass))
                self.sim.setFloatProperty(obj_handle, "bullet.friction", 0.6)
                break
        return obj_handle

    def place_target_pos_randomly_half_area(self):
        spawn_area_length = self.max_y - self.min_y
        box_size, _ = self.sim.getShapeBB(self.box)
        limit = (box_size[1] / 2) + 0.01  # To give some space between the box and the objects
        half = None
        if random.uniform(0, 1) <= 0.5:
            half = 0    # Right
            y_coord = self.min_y + random.uniform(0, (spawn_area_length / 2) - limit)
        else:
            half = 1    # Left
            y_coord = self.min_y + (spawn_area_length / 2) + random.uniform(limit, spawn_area_length / 2)
        target_pos = self.sim.getObjectPosition(self.box, self.reference_frame)
        x = random.uniform(self.min_x, self.max_x)
        self.sim.setObjectPosition(self.box, [x, y_coord - (box_size[1] / 2), target_pos[2]], self.reference_frame)
        return half

    def place_target_pos_randomly(self, clean_and_place=False):
        spawn_area_length = self.max_y - self.min_y
        box_size, _ = self.sim.getShapeBB(self.box)
        if self.obj_handle is not None:
            obj_size, _ = self.sim.getShapeBB(self.obj_handle)
            limit = obj_size[1] / 2 + box_size[1] / 2
            if self.sim.getObjectPosition(self.obj_handle, self.reference_frame)[1] <= self.min_y + (spawn_area_length / 2):
                y_coord = self.min_y + (spawn_area_length / 2) + random.uniform(limit, spawn_area_length / 2)
            else:
                y_coord = self.min_y + random.uniform(0, (spawn_area_length / 2) - limit)
        else:
            y_coord = random.uniform(self.min_y, self.max_y)
        target_pos = self.sim.getObjectPosition(self.box, self.reference_frame)
        if clean_and_place:
            x = random.uniform(self.min_x_box_clean_and_place, self.max_x)
        else:
            x = random.uniform(self.min_x, self.max_x)
        self.sim.setObjectPosition(self.box, [x, y_coord - (box_size[1] / 2), target_pos[2]], self.reference_frame)

    def init_pick_and_place(self, randomize_pos=True, obj=0):
        if randomize_pos:
            self.obj_handle = self.spawn_random_object(obj=obj)
            self.obj_starting_z = self.sim.getObjectPosition(self.obj_handle, self.reference_frame)[2]
            self.place_target_pos_randomly()
        else:
            self.obj_handle = self.spawn_default_cylinder()
            self.place_target_pos_default()

    def init_clean_and_place(self, obstacles=0, object_to_grab=2, n_objects=2, randomize_pos=True):
        if randomize_pos:
            self.half = self.place_target_pos_randomly_half_area()
            self.calculate_half_area_grid()
            for i in range(n_objects):
                obj_handle = self.spawn_random_object_half_area_grid(self.half, obstacles, obstacle=True)
                if obj_handle is not None:
                    self.objects.append(obj_handle)
            # Object to grab
            self.objects.append(self.spawn_random_object_half_area_grid(self.half, object_to_grab, obstacle=False))
            #print(self.obj_grid_dict)
        else:
            pass

    def reset(self, record=False):
        self.coppeliasim_client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.coppeliasim_client.getObject('sim')
        self.robot_model = 'Franka' if self.sim.getObject('/Franka', {'noError': True}) > 0 else 'LBRiiwa7R800'
        self.gripper_model = 'FrankaGripper' if self.sim.getObject('/FrankaGripper', {'noError': True}) > 0 else 'ROBOTIQ85'
        self.hand_base_handle = self.sim.getObject(f'/{self.robot_model}/connection')
        self.reference_frame = self.sim.getObject('/RobotWorldFrame')
        self.object_target = self.sim.getObject('/Box/ObjectTarget')
        self.target_zone = self.sim.getObject('/Box/TargetZone')
        self.land_zone = self.sim.getObject('/Box/LandZone')
        self.box = self.sim.getObject('/Box')
        self.drop_point = self.sim.getObject('/DropPoint')

        self.obj_handle = None

        self.end_effector = self.sim.getObject(f'/{self.robot_model}/EndEffector')
        
        self.target_handle = self.sim.getObject('/Target')
        self.ik_target = self.sim.getObject('/IKTarget')

        self.right_visible = 'rightPad_visible' if self.gripper_model == 'FrankaGripper' else 'RfingerTipVisible'
        self.left_visible = 'leftPad_visible' if self.gripper_model == 'FrankaGripper' else 'LfingerTipVisible'

        self.right_finger = self.sim.getObject(f'/{self.robot_model}/{self.gripper_model}/{self.right_visible}')
        self.left_finger = self.sim.getObject(f'/{self.robot_model}/{self.gripper_model}/{self.left_visible}')
        self.sim.setProperty(self.right_finger, 'collidable', True)
        self.sim.setProperty(self.left_finger, 'collidable', True)

        if record:
            self.sim.setBoolParam(self.sim.boolparam_video_recording_triggered, True)
        self.phase = 0
        self.collision_steps = 0
        self.sim.setStepping(True)