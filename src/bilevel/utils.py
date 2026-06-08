import numpy as np
import torch

def connect(gui=1):
    import importlib
    from pybullet_utils import bullet_client
    if gui:
        p = bullet_client.BulletClient(connection_mode=bullet_client.pybullet.GUI)
    else:
        p = bullet_client.BulletClient(connection_mode=bullet_client.pybullet.DIRECT)
        try:
            egl_spec = importlib.util.find_spec("eglRenderer")
            if egl_spec and egl_spec.origin:
                p.loadPlugin(egl_spec.origin, "_eglRendererPlugin")
            else:
                p.loadPlugin("eglRendererPlugin")
        except Exception:
            p.loadPlugin("eglRendererPlugin")
    return p


def create_object(p, obj_type, size, position, rotation=[0, 0, 0], mass=1,
                  color=None, with_link=False):
    collisionId = -1
    visualId = -1

    if obj_type == p.GEOM_SPHERE:
        collisionId = p.createCollisionShape(shapeType=obj_type, radius=size[0])
        if color == "random":
            color = np.random.rand(3).tolist() + [1]
            visualId = p.createVisualShape(shapeType=obj_type, radius=size[0], rgbaColor=color)
        elif color is not None:
            visualId = p.createVisualShape(shapeType=obj_type, radius=size[0], rgbaColor=color)

    elif obj_type in [p.GEOM_CAPSULE, p.GEOM_CYLINDER]:
        collisionId = p.createCollisionShape(shapeType=obj_type, radius=size[0], height=size[1])
        if color == "random":
            color = np.random.rand(3).tolist() + [1]
            visualId = p.createVisualShape(shapeType=obj_type, radius=size[0], length=size[1], rgbaColor=color)
        elif color is not None:
            visualId = p.createVisualShape(shapeType=obj_type, radius=size[0], length=size[1], rgbaColor=color)

    elif obj_type == p.GEOM_BOX:
        collisionId = p.createCollisionShape(shapeType=obj_type, halfExtents=size)
        if color == "random":
            color = np.random.rand(3).tolist() + [1]
            visualId = p.createVisualShape(shapeType=obj_type, halfExtents=size, rgbaColor=color)
        elif color is not None:
            visualId = p.createVisualShape(shapeType=obj_type, halfExtents=size, rgbaColor=color)

    elif obj_type == "random":
        obj = "%03d" % np.random.randint(1000)
        obj_id = p.loadURDF(f"random_urdfs/{obj}/{obj}.urdf", basePosition=position,
                            baseOrientation=p.getQuaternionFromEuler(rotation))
        return obj_id

    if with_link:
        obj_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=-1,
                                   basePosition=position, baseOrientation=p.getQuaternionFromEuler(rotation),
                                   linkMasses=[mass], linkCollisionShapeIndices=[collisionId],
                                   linkVisualShapeIndices=[visualId], linkPositions=[[0, 0, 0]],
                                   linkOrientations=[[0, 0, 0, 1]], linkInertialFramePositions=[[0, 0, 0]],
                                   linkInertialFrameOrientations=[[0, 0, 0, 1]], linkParentIndices=[0],
                                   linkJointTypes=[p.JOINT_FIXED], linkJointAxis=[[0, 0, 0]])
    else:
        obj_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collisionId,
                                   baseVisualShapeIndex=visualId,
                                   basePosition=position,
                                   baseOrientation=p.getQuaternionFromEuler(rotation))
        p.changeDynamics(obj_id, -1, rollingFriction=0.0005, spinningFriction=0.001)

    return obj_id


def create_tabletop(p):
    objects = {}
    objects["base"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.15, 0.15, 0.2],
                                    position=[0., 0., 0.2], color=[0.5, 0.5, 0.5, 1.0], with_link=True)
    objects["table"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.7, 1, 0.2],
                                     position=[0.9, 0, 0.2], color=[0.9, 0.9, 0.9, 1.0])
    objects["wall1"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.7, 0.01, 0.05],
                                     position=[0.9, -1, 0.45], color=[0.4, 0.4, 1.0, 1.0])
    objects["wall2"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.7, 0.01, 0.05],
                                     position=[0.9, 1, 0.45], color=[0.4, 0.4, 1.0, 1.0])
    objects["wall3"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.01, 1, 0.05],
                                     position=[0.2, 0., 0.45], color=[0.4, 0.4, 1.0, 1.0])
    objects["wall4"] = create_object(p, p.GEOM_BOX, mass=0, size=[0.01, 1, 0.05],
                                     position=[1.6, 0., 0.45], color=[0.4, 0.4, 1.0, 1.0])
    return objects


def get_image(p, eye_position, target_position, up_vector, height, width):
    viewMatrix = p.computeViewMatrix(cameraEyePosition=eye_position,
                                     cameraTargetPosition=target_position,
                                     cameraUpVector=up_vector)
    projectionMatrix = p.computeProjectionMatrixFOV(fov=45, aspect=1.0, nearVal=0.75, farVal=5.5)
    _, _, rgb, depth, seg = p.getCameraImage(height=height, width=width, viewMatrix=viewMatrix,
                                             projectionMatrix=projectionMatrix)
    return rgb, depth, seg


def mask_unknown(x, threshold=0.1):
    mask = x.std(dim=0) > threshold
    x_mu = x.mean(dim=0).round().char()
    x_mu[mask] = 3
    return x_mu


def state_to_symbol(state, model):
    N_EPS = 1000
    state = state.detach().clone().unsqueeze(0).to(model.device)
    state = state.repeat(N_EPS, 1, 1).reshape(-1, *state.shape[1:])
    mask = torch.ones(state.shape[0], state.shape[1])
    z = model.encode(state)
    r = model.attn_weights(state, mask)
    return z, r


def state_to_problem(init_state, final_state, model):
    z, r = state_to_symbol(init_state, model)
    zn, rn = state_to_symbol(final_state, model)
    z, r = mask_unknown(z), mask_unknown(r)
    zn, rn = mask_unknown(zn), mask_unknown(rn)
    return z, r, zn, rn