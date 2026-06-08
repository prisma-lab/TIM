import pybullet
import pybullet_data
import numpy as np
import utils
import manipulators
import pybullet
import pybullet_data
import numpy as np

class GenericEnv:
    def __init__(self, gui=0, seed=None):
        self._p = utils.connect(gui)
        self.gui = gui
        self.reset(seed=seed)

    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

        self._p.resetSimulation()
        self._p.configureDebugVisualizer(self._p.COV_ENABLE_GUI, 0)
        self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._p.setGravity(0, 0, -9.807)
        self._p.loadURDF("plane.urdf")

        self.env_dict = utils.create_tabletop(self._p)
        self.agent = manipulators.Manipulator(p=self._p, path="ur10e/ur10e.urdf", position=[0., 0., 0.4], ik_idx=30)
        base_constraint = self._p.createConstraint(parentBodyUniqueId=self.env_dict["base"], parentLinkIndex=0,
                                                   childBodyUniqueId=self.agent.id, childLinkIndex=-1,
                                                   jointType=self._p.JOINT_FIXED, jointAxis=(0, 0, 0),
                                                   parentFramePosition=(0, 0, 0),
                                                   childFramePosition=(0.0, 0.0, -0.2),
                                                   childFrameOrientation=(0, 0, 0, 1))
        self._p.changeConstraint(base_constraint, maxForce=10000)
     
        mimic_constraint = self._p.createConstraint(self.agent.id, 28, self.agent.id, 29,
                                                    jointType=self._p.JOINT_GEAR,
                                                    jointAxis=[1, 0, 0],
                                                    parentFramePosition=[0, 0, 0],
                                                    childFramePosition=[0, 0, 0])
        self._p.changeConstraint(mimic_constraint, gearRatio=-1, erp=0.1, maxForce=50)

    def init_agent_pose(self, t=None, sleep=False, traj=False):
        angles = [-0.294, -1.950, 2.141, -2.062, -1.572, 1.277]
        self.agent.set_joint_position(angles, t=t, sleep=sleep, traj=traj)

    def state_obj_poses(self):
        N_obj = len(self.obj_dict)
        pose = np.zeros((N_obj, 7), dtype=np.float32)
        for i in range(N_obj):
            position, quaternion = self._p.getBasePositionAndOrientation(self.obj_dict[i])
            pose[i][:3] = position
            pose[i][3:] = quaternion
        return pose

    def get_contact_graph(self):
        N_obj = len(self.obj_dict)
        contact_graph = np.zeros((N_obj, N_obj), dtype=int)
        reverse_obj_dict = {v: k for k, v in self.obj_dict.items()}
        for i in range(N_obj):
            contacts = self._p.getContactPoints()
            for contact in contacts:
                if (contact[1] in reverse_obj_dict) and (contact[2] in reverse_obj_dict):
                    contact_graph[reverse_obj_dict[contact[1]], reverse_obj_dict[contact[2]]] = 1
                    contact_graph[reverse_obj_dict[contact[2]], reverse_obj_dict[contact[1]]] = 1

        for i in range(contact_graph.shape[0]):
            neighbors = np.where(contact_graph[i] == 1)[0]
            for ni in neighbors:
                for nj in neighbors:
                    contact_graph[ni, nj] = 1
            contact_graph[i, i] = 1
        return contact_graph

    def _step(self, count=1):
        for _ in range(count):
            self._p.stepSimulation()

    def _eye_from_yaw_pitch_dist(self, target, yaw_deg, pitch_deg, dist):

        yaw = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        cx = np.cos(pitch) * np.cos(yaw)
        cy = np.cos(pitch) * np.sin(yaw)
        cz = np.sin(pitch)
        eye = [target[0] - dist * cx, target[1] - dist * cy, target[2] - dist * cz]
        return eye
        
    def _robot_view_camera(self):
        yaw_deg = 180           
        pitch_deg = -50         
        dist = 2.0             
        target_position = [0.95, 0.00, 0.48] 
        eye_position = self._eye_from_yaw_pitch_dist(target_position, yaw_deg, pitch_deg, dist)
        up_vector = [0, 0, 1]
        return eye_position, target_position, up_vector

    def get_robot_view_image(self, height=512, width=512):
        eye, target, up = self._robot_view_camera()
        rgb, depth, seg = utils.get_image(
            p=self._p,
            eye_position=eye,
            target_position=target,
            up_vector=up,
            height=height,
            width=width,
        )
        return rgb[:, :, :3], depth, seg

    def _cache_original_rgba(self, body_id):
       
        vs = self._p.getVisualShapeData(body_id)
        for entry in vs:
            if entry[1] == -1:        
                return list(entry[7]) 
    
        return [1, 1, 1, 1]

    def _set_rgba(self, body_id, rgba):
        self._p.changeVisualShape(body_id, -1, rgbaColor=rgba)

    def _apply_state_rows(self, state, use_orientation=True):

        N = len(self.obj_dict)
        assert state.shape[0] >= N, f"State rows ({state.shape[0]}) < num objects ({N})"
        has_quat = (state.shape[1] >= 7) and use_orientation
        for i in range(N):
            obj_id = self.obj_dict[i]
            pos = state[i, :3].tolist()
            if has_quat:
                quat = state[i, 3:7].tolist()
            else:
                _, quat = self._p.getBasePositionAndOrientation(obj_id)
            self._p.resetBasePositionAndOrientation(obj_id, pos, quat)

    def _detect_changed_mask(self, prev_state, next_state, pos_eps=0.05,
                              use_orientation=False, quat_eps=1e-3):
        N = min(prev_state.shape[0], next_state.shape[0], len(self.obj_dict))
        changed = np.zeros(N, dtype=bool)
        dp = np.linalg.norm(next_state[:, :3] - prev_state[:, :3], axis=1)
        changed |= (dp > pos_eps)



        
        if use_orientation and prev_state.shape[1] >= 7 and next_state.shape[1] >= 7:
            dq = np.linalg.norm(next_state[:N, 3:7] - prev_state[:N, 3:7], axis=1)
            changed |= (dq > quat_eps)
        return changed

    def visualize_and_save_verifier_predictions(
        self,
        predicted_states,
        out_dir,
        prefix="problem",
        transparent_alpha=0.35,
        pos_eps=5e-2,
        use_orientation_change=False,
        settle_steps=30,
        image_size=(512, 512),
        save_images=True,
    ):

        import os
        os.makedirs(out_dir, exist_ok=True)

        if hasattr(self, 'initial_colors') and self.initial_colors:
             original_rgba = self.initial_colors
        else:
             original_rgba = {i: self._cache_original_rgba(self.obj_dict[i]) for i in range(len(self.obj_dict))}


        try:
            self.agent.open_gripper(0.25, sleep=False)
        except Exception:
            pass

        H, W = image_size
        frames = []
        prev_state = None
        init_state = predicted_states[0]
        for t, state in enumerate(predicted_states):
        
            self._apply_state_rows(state, use_orientation=False)
            for _ in range(settle_steps):
                self._p.stepSimulation()

    
            if prev_state is None:
                changed = np.zeros(len(self.obj_dict), dtype=bool)
            else:
                changed = self._detect_changed_mask(init_state, state, pos_eps,
                                                    use_orientation_change)

       
            for i in range(len(self.obj_dict)):
                rgba = original_rgba[i].copy()
                rgba[3] = transparent_alpha if changed[i] else 1.0
                self._set_rgba(self.obj_dict[i], rgba)

      
            rgb, _, _ = self.get_robot_view_image(height=H, width=W)
            frames.append(rgb.copy())

            if save_images:
                import imageio.v2 as imageio_png  
                imageio_png.imwrite(os.path.join(out_dir, f"{prefix}_step_{t:03d}.png"), rgb)

            prev_state = state

   
        for i in range(len(self.obj_dict)):
            self._set_rgba(self.obj_dict[i], original_rgba[i])

        return frames


    def __del__(self):
        self._p.disconnect()


class BlocksWorld(GenericEnv):
    def __init__(self, gui=0, seed=None, min_objects=2, max_objects=5):
        self.min_objects = min_objects
        self.max_objects = max_objects
        self.num_objects = None
        super(BlocksWorld, self).__init__(gui=gui, seed=seed)

    def reset(self, seed=None):
        super(BlocksWorld, self).reset(seed=seed)

        self.obj_dict = {}
        self.init_agent_pose(t=1)
        self.init_objects()
        self._step(40)
        self.agent.open_gripper(1, sleep=True)

    def delete_objects(self):
        for key in self.obj_dict:
            obj_id = self.obj_dict[key]
            self._p.removeBody(obj_id)
        self.obj_dict = {}

    def reset_objects(self):
        self.delete_objects()
        self.init_objects()
        self._step(240)

    def reset_objects_manual(self):
        self.delete_objects()
        self.init_objects_manual()
        self._step(240)

    def reset_object_poses(self):
        for key in self.obj_dict:
            x = np.random.uniform(0.5, 1.0)
            y = np.random.uniform(-0.4, 0.4)
            z = np.random.uniform(0.6, 0.65)
            quat = pybullet.getQuaternionFromEuler(np.random.uniform(0, 90, (3,)).tolist())

            self._p.resetBasePositionAndOrientation(self.obj_dict[key], [x, y, z], quat)
        self._step(240)

    def init_objects(self):
        self.num_objects = np.random.randint(self.min_objects, self.max_objects+1)
        for i in range(self.num_objects):
            obj_type = np.random.choice([self._p.GEOM_BOX], p=[1])
            x = np.random.uniform(0.5, 1.0)
            y = np.random.uniform(-0.4, 0.4)
            z = np.random.uniform(0.6, 0.65)
            size = np.random.uniform(0.015, 0.035, (3,)).tolist()
            rotation = np.random.uniform(0, 90, (3,)).tolist()

            if obj_type == self._p.GEOM_BOX:
                if np.random.rand() < 0.4:
                    size = [np.random.uniform(0., 0.2), np.random.uniform(0.01, 0.015),
                            np.random.uniform(0.015, 0.025)]
            self.obj_dict[i] = utils.create_object(p=self._p, obj_type=obj_type, size=size, position=[x, y, z],
                                                   rotation=rotation, color="random", mass=0.1)
            

    def state(self):
        rgb, depth, seg = utils.get_image(p=self._p, eye_position=[1.2, 0.0, 1.6], target_position=[0.8, 0., 0.4],
                                          up_vector=[0, 0, 1], height=512, width=512)
        return rgb[:, :, :3], depth, seg

    def step(self, from_obj_id, to_obj_id, sleep=False):
        from_pos, from_quat = self._p.getBasePositionAndOrientation(from_obj_id)
        to_pos, to_quat = self._p.getBasePositionAndOrientation(to_obj_id)
        to_pos = to_pos[:2] + (0.75,)
        traj_time = 0.5
        self.agent.set_cartesian_position(from_pos[:2]+(0.75,),
                                          orientation=self._p.getQuaternionFromEuler([np.pi, 0, 0]),
                                          t=traj_time,
                                          traj=True,
                                          sleep=sleep)
        self.agent.set_cartesian_position(from_pos,
                                          orientation=self._p.getQuaternionFromEuler([np.pi, 0, 0]),
                                          t=traj_time,
                                          traj=True,
                                          sleep=sleep)
        self.agent.close_gripper(traj_time, sleep=sleep)
        self.agent.set_cartesian_position(from_pos[:2]+(0.75,),
                                          orientation=self._p.getQuaternionFromEuler([np.pi, 0, 0]),
                                          t=traj_time,
                                          traj=True,
                                          sleep=sleep)
        self.agent.set_cartesian_position(to_pos,
                                          orientation=self._p.getQuaternionFromEuler([np.pi, 0, 0]),
                                          t=traj_time,
                                          traj=True,
                                          sleep=sleep)
        before_pose = self.state_obj_poses()
        self.agent.open_gripper(traj_time, sleep=sleep)
        self.init_agent_pose(t=1.0, sleep=sleep)
        after_pose = self.state_obj_poses()
        effect = after_pose - before_pose
        return effect


class BlocksWorld_v4(BlocksWorld):
    def __init__(self, x_area=0.5, y_area=0.8, **kwargs):
        self.traj_t = 1.5
        self.x_init = 0.5
        self.y_init = -0.4
        self.x_final = self.x_init + x_area
        self.y_final = self.y_init + y_area
        self.ds = 0.075
        self.obj_types = {}
        self.sizes = [[0.025, 0.025, 0.025], [0.025, 0.125, 0.025]]
        self.colors = [[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0],
                       [1.0, 1.0, 0.0, 1.0], [1.0, 0.0, 1.0, 1.0], [0.0, 1.0, 1.0, 1.0]]
        self.obj_enc = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.obj_dict = {}
        self.initial_colors = {} 
        super(BlocksWorld_v4, self).__init__(**kwargs)

    def init_objects(self):
        self.initial_colors = {}
        self.obj_types = {}
        obj_ids = []
        self.num_objects = np.random.randint(self.min_objects, self.max_objects + 1)
        obj_types = np.random.choice([0, 1], size=(self.num_objects,), replace=True)
        colors_indices = np.random.choice([0, 1, 2, 3, 4, 5], size=(self.num_objects,), replace=True)
        colors = [self.colors[c] for c in colors_indices]
        id_to_color_map = {}
        i = 0
        positions = []
        while i < self.num_objects:
            obj_type = obj_types[i]
            x = np.random.uniform(self.x_init, self.x_final)
            y = np.random.uniform(self.y_init, self.y_final)
            z = 0.43
            pos = np.array([x, y])
            if len(positions) > 0 and np.any(np.linalg.norm(np.stack(positions) - pos, axis=-1) < 0.20):
                continue
            mass = 0.1 if obj_type == 0 else 0.4
            o_id = utils.create_object(p=self._p, obj_type=self._p.GEOM_BOX,
                                       size=self.sizes[obj_type], position=[x, y, z], rotation=[0, 0, 0],
                                       mass=mass, color=colors[i])
            id_to_color_map[o_id] = colors[i]
            self.obj_types[o_id] = obj_type
            positions.append(pos)
            obj_ids.append(o_id)
            i += 1
        for i, o_id in enumerate(sorted(obj_ids)):
            self.obj_dict[i] = o_id
            self.initial_colors[i] = id_to_color_map[o_id]

    def init_objects_manual(self):
        self.initial_colors = {}
        self.obj_types = {}
        obj_ids = []
        self.num_objects = np.random.randint(self.min_objects, self.max_objects + 1)
        obj_types = [1] + [0] * (self.num_objects - 1)
        np.random.shuffle(obj_types)
        colors_indices = np.random.choice([0, 1, 2, 3, 4, 5], size=(self.num_objects,), replace=True)
        colors = [self.colors[c] for c in colors_indices]
        id_to_color_map = {}
        i = 0
        positions = []
        while i < self.num_objects:
            obj_type = obj_types[i]
            x = np.random.uniform(self.x_init, self.x_final)
            y = np.random.uniform(self.y_init, self.y_final)
            z = 0.43
            pos = np.array([x, y])
            if len(positions) > 0 and np.any(np.linalg.norm(np.stack(positions) - pos, axis=-1) < 0.20):
                continue
            mass = 0.1 if obj_type == 0 else 0.4
            o_id = utils.create_object(p=self._p, obj_type=self._p.GEOM_BOX,
                                       size=self.sizes[obj_type], position=[x, y, z], rotation=[0, 0, 0],
                                       mass=mass, color=colors[i])
            id_to_color_map[o_id] = colors[i]
            self.obj_types[o_id] = obj_type
            positions.append(pos)
            obj_ids.append(o_id)
            self._p.addUserDebugText(str(i), [0, 0, 0.1], [0, 0, 0], 1, 0, parentObjectUniqueId=o_id)
            i += 1
        for i, o_id in enumerate(sorted(obj_ids)):
            self.obj_dict[i] = o_id
            self.initial_colors[i] = id_to_color_map[o_id]
    
    def step(self, obj1_id, obj2_id, dx1, dy1, dx2, dy2, rotated_grasp,
             rotated_release, sleep=False, get_images=False):
        eye_position, target_position, up_vector = self._robot_view_camera()
        images = []
        if get_images:
            images.append(utils.get_image(self._p, eye_position=eye_position, target_position=target_position,
                                          up_vector=up_vector, height=512, width=512)[0])

        obj1_loc, _ = self._p.getBasePositionAndOrientation(self.obj_dict[obj1_id])
        obj2_loc, _ = self._p.getBasePositionAndOrientation(self.obj_dict[obj2_id])
        grasp_angle1 = [np.pi, 0, 0]
        grasp_angle2 = [np.pi, 0, 0]
        if rotated_grasp:
            grasp_angle1[2] = np.pi/2
        if rotated_release:
            grasp_angle2[2] = np.pi/2
        quat1 = self._p.getQuaternionFromEuler(grasp_angle1)
        quat2 = self._p.getQuaternionFromEuler(grasp_angle2)
        obj1_loc = list(obj1_loc)
        obj2_loc = list(obj2_loc)
        obj1_loc[0] += dx1 * self.ds
        obj2_loc[0] += dx2 * self.ds
        obj1_loc[1] += dy1 * self.ds
        obj2_loc[1] += dy2 * self.ds
        from_top_pos = obj1_loc.copy()
        from_top_pos[2] = 0.9
        to_top_pos = obj2_loc.copy()
        to_top_pos[2] = 0.9

        state1 = self.state()
        self.agent.set_cartesian_position(from_top_pos, orientation=quat1, t=self.traj_t, traj=True, sleep=sleep)
        self.agent.move_in_cartesian(obj1_loc, orientation=quat1, t=self.traj_t, sleep=sleep)
        if get_images:
            images.append(utils.get_image(self._p, eye_position=eye_position, target_position=target_position,
                                          up_vector=up_vector, height=512, width=512)[0])
        self.agent.close_gripper(self.traj_t, sleep=sleep)
        self.agent.move_in_cartesian(from_top_pos, orientation=quat1, t=self.traj_t, sleep=sleep)
        state2 = self.state()
        self.agent.move_in_cartesian(to_top_pos, orientation=quat2, t=self.traj_t, sleep=sleep, ignore_force=True)
        self.agent._waitsleep(0.3, sleep=sleep)
        if get_images:
            images.append(utils.get_image(self._p, eye_position=eye_position, target_position=target_position,
                                          up_vector=up_vector, height=512, width=512)[0])
        state3 = self.state()
        self.agent.move_in_cartesian(obj2_loc, orientation=quat2, t=self.traj_t, sleep=sleep)
        self.agent._waitsleep(0.5, sleep=sleep)
        self.agent.open_gripper()
        self.agent.move_in_cartesian(to_top_pos, orientation=quat2, t=self.traj_t, sleep=sleep)
        if get_images:
            images.append(utils.get_image(self._p, eye_position=eye_position, target_position=target_position,
                                          up_vector=up_vector, height=512, width=512)[0])
        self.init_agent_pose(t=1.0, sleep=sleep)
        self.agent._waitsleep(2)
        state4 = self.state()
        delta_pos1 = state2[:, :3] - state1[:, :3]
        delta_quat1 = np.stack([self._p.getDifferenceQuaternion(q2, q1) for q1, q2 in zip(state1[:, 3:7], state2[:, 3:7])])
        delta_pos2 = state4[:, :3] - state3[:, :3]
        delta_quat2 = np.stack([self._p.getDifferenceQuaternion(q2, q1) for q1, q2 in zip(state3[:, 3:7], state4[:, 3:7])])
        effect = np.concatenate([delta_pos1, delta_quat1, delta_pos2, delta_quat2], axis=-1)
        if get_images:
            return state1, effect, images
        return state1, effect

    def state(self):
        poses = self.state_obj_poses()
        obj_types = np.array([self.obj_types[self.obj_dict[i]] for i in range(len(self.obj_dict))])
        state = np.concatenate([poses, self.obj_enc[obj_types]], axis=1)
        return state

    def full_random_action(self):
        obj1_id, obj2_id = np.random.choice(list(self.obj_dict.keys()), size=(2,), replace=True)
        dy1 = np.random.choice([-1, 0, 1])
        dy2 = np.random.choice([-1, 0, 1])
        return [obj1_id, obj2_id, 0, dy1, 0, dy2, 1, 1]
    
    def get_object_types(self):
        obj_types = np.array([self.obj_types[self.obj_dict[i]] for i in range(len(self.obj_dict))])
        return obj_types
