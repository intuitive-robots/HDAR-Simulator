import numpy as np
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceController
from simpub.xr_device.meta_quest3 import MetaQuest3
from scipy.spatial.transform import Rotation as R


class MetaQuest3Controller_right(CartPosQuatImpedenceController):

    def __init__(self, device):
        super().__init__()
        self.device: MetaQuest3 = device

    def getControl(self, robot):
        input_data = self.device.get_input_data()
        if input_data is not None:
            desired_pos = input_data.right_pos
            pos_offset = np.array([-0.2,0.0,0.0])          
            desired_quat = input_data.right_rot
            if desired_quat != [0,0,0,0]:
                q = desired_quat
                rot = R.from_quat(q)
                rot = rot.as_euler('xyz')
                rot = [rot[2], rot[1], rot[0]]
                desired_quat = R.from_euler("xyz", rot).as_quat()
                q_rot1=np.array([0,0,0,1])
                q_rot2=np.array([0,0,1,0])
                r0=R.from_quat([desired_quat[3],desired_quat[0],desired_quat[1],desired_quat[2]])
                r1=R.from_quat([q_rot1[3],q_rot1[0],q_rot1[1],q_rot1[2]])
                r2=R.from_quat([q_rot2[3],q_rot2[0],q_rot2[1],q_rot2[2]])
                r_mi1=r0*r1
                r_mi2=r_mi1*r2
                r_combined=r_mi2
                q_combined=r_combined.as_quat()
                q_final=np.array([-q_combined[1],-q_combined[2],q_combined[3],q_combined[0]])
                               
    
            desired_pos_local = robot._localize_cart_pos(desired_pos)+pos_offset
            desired_quat_local = q_final
            if input_data.right_index_trigger:
                robot.close_fingers(duration=0.0)
            else:
                robot.open_fingers()
            self.setSetPoint(np.hstack((desired_pos_local, desired_quat_local)))
        return super().getControl(robot)
