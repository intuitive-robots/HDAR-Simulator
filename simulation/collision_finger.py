import numpy as np
import mujoco
import os
import math

class Collision_finger:
    def __init__(self, vt_scene, target_pairs1, target_pairs2):
        self.vt_scene=vt_scene
        self.target_pairs1=target_pairs1
        self.target_pairs2=target_pairs2
        self.replace_l = 0
        self.replace_r = 0

    def check_collision(self, target_pairs, filename):
        collision_list = []
        for i in range(self.vt_scene.data.ncon):
            contact = self.vt_scene.data.contact[i]
            pos = np.array(contact.pos[:3])
            geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            force = np.zeros(6)
            mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
            time = self.vt_scene.data.time
            if (geom1_name, geom2_name) in target_pairs or (geom2_name, geom1_name) in target_pairs:
                collision_list.append({
                    'geom1': geom1_name,
                    'geom2': geom2_name,
                    'position': pos.astype(float),
                    'force': force[:3].astype(float),
                    'time': time,
                    'number': i
                })
                with open(filename, 'a', newline='') as file:
                    file.write(f"Collision object:{geom1_name} with {geom2_name} at {pos}, Force: {force[:3]}, time:{time}, number:{i}\n")
        return collision_list

    def calculate_resultant_force(self, collision_list, filename, replace_attr):
        collision_resultant = {}
        for collision in collision_list:
            geom1 = collision['geom1']
            geom2 = collision['geom2']
            pos = collision['position']
            force = collision['force']
            time = collision['time']
            time_key = round(time, 4)
            geom_pair = tuple(sorted((geom1, geom2)))
            if (time_key, geom_pair) not in collision_resultant:
                collision_resultant[(time_key, geom_pair)] = {
                    'geom1': geom1,
                    'geom2': geom2,
                    'position_sum': pos,
                    'force': force,
                    'number': 1
                }
            else:
                collision_resultant[(time_key, geom_pair)]['force'] += force
                collision_resultant[(time_key, geom_pair)]['position_sum'] += pos
                collision_resultant[(time_key, geom_pair)]['number'] += 1

        with open(filename, 'a', newline='') as file:
            for (time_key, geom_pair), data in collision_resultant.items():
                avg_position = data['position_sum'] / data['number']
                force = data['force']
                force_x, force_y, force_z = force
                resultant_force = math.sqrt(force_x**2 + force_y**2 + force_z**2)
                replace = min(1, resultant_force / 10)
                file.write(f"Collision object:{data['geom1']} with {data['geom2']} at {avg_position}, Force: {force}, Replace Value: {replace}, time:{time_key}\n")
                
                # Update replace value
                setattr(self, replace_attr, replace)

    def process_collisions(self):
        # left
        collision_finger1_list = self.check_collision(self.target_pairs1, 'collision_finger1.txt')
        self.calculate_resultant_force(collision_finger1_list, 'collision_finger1_resultant.txt', 'replace_l')

        # right
        collision_finger2_list = self.check_collision(self.target_pairs2, 'collision_finger2.txt')
        self.calculate_resultant_force(collision_finger2_list, 'collision_finger2_resultant.txt', 'replace_r')

        return self.replace_l, self.replace_r
    
