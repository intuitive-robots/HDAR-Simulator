from alr_sim.sims.universal_sim.PrimitiveObjects import PrimitiveObject


def mj2unity_pos(pos):
    return [-pos[1], pos[2], pos[0]]


def mj2unity_quat(quat):
    # note that the order is "[x, y, z, w]"
    return [quat[2], -quat[3], -quat[1], quat[0]]


def mj2unity_size(obj: PrimitiveObject):
    if obj.type == PrimitiveObject.Shape.BOX:
        return [obj.size[1] * 2, obj.size[2] * 2, obj.size[0] * 2]
    elif obj.type == PrimitiveObject.Shape.SPHERE:
        return [obj.size[1], obj.size[2], obj.size[0]]
    elif obj.type == PrimitiveObject.Shape.CYLINDER:
        return [obj.size[1], obj.size[2] * 2, obj.size[0]]
    else:
        return [1, 1, 1]


def unity2mj_pos(pos):
    return [-pos[1], pos[2], pos[0]]


def unity2mj_quat(quat):
    # note that the order is "[x, y, z, w]"
    return [quat[2], -quat[3], -quat[1], quat[0]]