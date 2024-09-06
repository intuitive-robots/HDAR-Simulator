import numpy as np
import mujoco
import os
import math




class Collision_finger:


    def __init__(self, vt_scene, target_pairs1, target_pairs2):
        self.vt_scene = vt_scene
        self.target_pairs1 = target_pairs1
        self.target_pairs2 = target_pairs2
        self.collision_l = 0
        self.collision_r = 0

    def check_collision(self, target_pairs):
        if self.vt_scene.data is None or self.vt_scene.data.ncon <= 0:
            return False
        
        for i in range(self.vt_scene.data.ncon):
            # Perform index check to ensure i is within range
            if i >= len(self.vt_scene.data.contact):
                continue

            contact = self.vt_scene.data.contact[i]
            geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            if (geom1_name, geom2_name) in target_pairs or (geom2_name, geom1_name) in target_pairs:
                return True

        return False
    
    def get_collisions(self):
        if self.check_collision(self.target_pairs1):
            self.collision_l = 1 
        
        if self.check_collision(self.target_pairs2):
            self.collision_r = 1 
        
        return self.collision_l, self.collision_r
    

class Collision_aim:

    def __init__(self, vt_scene, target_pairs):
        
        self.vt_scene = vt_scene
        self.target_pairs = target_pairs
        self.contact_aim = 0
    
    def check_collision(self, target_pairs):
        if self.vt_scene.data is None or self.vt_scene.data.ncon <= 0:
            return False
        
        for i in range(self.vt_scene.data.ncon):
            contact = self.vt_scene.data.contact[i]
            geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if (geom1_name, geom2_name) in target_pairs or (geom2_name, geom1_name) in target_pairs:
                return True

        return False
    
    def aim_resultant_force(self):
        if self.check_collision(self.target_pairs):
            self.contact_aim = 1 
        return self.contact_aim



# class Collision_finger:


#     def __init__(self, vt_scene, target_pairs1, target_pairs2):
#         self.vt_scene = vt_scene
#         self.target_pairs1 = target_pairs1
#         self.target_pairs2 = target_pairs2
#         self.replace_l = 0
#         self.replace_r = 0

    # def check_collision(self, target_pairs):
    #     collision_list = []

    #     if self.vt_scene.data is None or self.vt_scene.data.ncon <= 0:
    #         return collision_list

    #     # Traverse the contact points
    #     for i in range(self.vt_scene.data.ncon):
    #         # Perform index check to ensure i is within range
    #         if i >= len(self.vt_scene.data.contact):
    #             continue

    #         contact = self.vt_scene.data.contact[i]
    #         pos = np.array(contact.pos[:3])
    #         geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
    #         geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
    #         force = np.zeros(6)
    #         mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
    #         time = self.vt_scene.data.time
    #         if (geom1_name, geom2_name) in target_pairs or (geom2_name, geom1_name) in target_pairs:
    #             collision_list.append({
    #                 'geom1': geom1_name,
    #                 'geom2': geom2_name,
    #                 'position': pos.astype(float),
    #                 'force': force[:3].astype(float),
    #                 'time': time,
    #                 'number': i
    #             })

    #     return collision_list

    # def calculate_resultant_force(self, collision_list, replace_attr):
    #     collision_resultant = {}
    #     for collision in collision_list:
    #         geom1 = collision['geom1']
    #         geom2 = collision['geom2']
    #         pos = collision['position']
    #         force = collision['force']
    #         time = collision['time']
    #         time_key = round(time, 4)
    #         geom_pair = tuple(sorted((geom1, geom2)))
    #         if (time_key, geom_pair) not in collision_resultant:
    #             collision_resultant[(time_key, geom_pair)] = {
    #                 'geom1': geom1,
    #                 'geom2': geom2,
    #                 'position_sum': pos,
    #                 'force': force,
    #                 'number': 1
    #             }
    #         else:
    #             collision_resultant[(time_key, geom_pair)]['force'] += force
    #             collision_resultant[(time_key, geom_pair)]['position_sum'] += pos
    #             collision_resultant[(time_key, geom_pair)]['number'] += 1

    #     if collision_resultant:
    #         for (time_key, geom_pair), data in collision_resultant.items():
    #             force = data['force']
    #             force_x, force_y, force_z = force
    #             resultant_force = math.sqrt(force_x**2 + force_y**2 + force_z**2)
    #             replace = min(1, resultant_force / 10)
                
    #             # Update replace value
    #             setattr(self, replace_attr, replace)
    #     else:
    #         # If no collision, set replace attribute to 0
    #         setattr(self, replace_attr, 0)

    # def get_collisions(self):
    #     # Left finger
    #     collision_finger1_list = self.check_collision(self.target_pairs1)
    #     self.calculate_resultant_force(collision_finger1_list, 'replace_l')

    #     # Right finger
    #     collision_finger2_list = self.check_collision(self.target_pairs2)
    #     self.calculate_resultant_force(collision_finger2_list, 'replace_r')

    #     return self.replace_l, self.replace_r

# class Collision_aim:
#     def __init__(self, vt_scene, target_pairs):
        
#         self.vt_scene = vt_scene
#         self.target_pairs = target_pairs

#     def check_collision_aim(self):
       
#         collision_aim_list = []
        
#         for i in range(self.vt_scene.data.ncon):
#             contact = self.vt_scene.data.contact[i]
#             pos = np.array(contact.pos[:3])
#             geom1_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
#             geom2_name = mujoco.mj_id2name(self.vt_scene.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
#             force = np.zeros(6)
#             mujoco.mj_contactForce(self.vt_scene.model, self.vt_scene.data, i, force)
#             time = self.vt_scene.data.time
            
#             if (geom1_name, geom2_name) in self.target_pairs or (geom2_name, geom1_name) in self.target_pairs:
#                 collision_aim_list.append({
#                     'geom1': geom1_name,
#                     'geom2': geom2_name,
#                     'position': pos.astype(float),
#                     'force': force[:3].astype(float),  # Only take the first three elements (force)
#                     'time': time,
#                     'number': i
#                 })
                
#         return collision_aim_list
    
#     def aim_resultant_force(self, replace_rb0_l=0, replace_rb0_r=0, replace_rb1_l=0, replace_rb1_r=0):
       
#         collision_aim_list = self.check_collision_aim()
#         collision_aim_resultant = {}
        
#         for collision in collision_aim_list:
#             geom1 = collision['geom1']
#             geom2 = collision['geom2']
#             pos = collision['position']
#             force = collision['force']
#             time = collision['time']
#             time_key = round(time, 4)
#             geom_pair = tuple(sorted((geom1, geom2)))
            
#             if (time_key, geom_pair) not in collision_aim_resultant:
#                 collision_aim_resultant[(time_key, geom_pair)] = {
#                     'geom1': geom1,
#                     'geom2': geom2,
#                     'position_sum': pos,
#                     'force': force,
#                     'newtime': time_key,
#                     'number': 1
#                 }
#             else:
#                 collision_aim_resultant[(time_key, geom_pair)]['force'] += force[:3]
#                 collision_aim_resultant[(time_key, geom_pair)]['position_sum'] += pos
#                 collision_aim_resultant[(time_key, geom_pair)]['number'] += 1
                
#         replaceaim = 0
#         for data in collision_aim_resultant.values():
#             # avg_position = data['position_sum'] / data['number']
#             force = data['force']
#             force_x, force_y, force_z = force
#             resultantforce = math.sqrt(force_x**2 + force_y**2 + force_z**2)
            
#             if any([replace_rb0_l, replace_rb0_r, replace_rb1_l, replace_rb1_r]):
#                 if resultantforce >= 10:
#                     replaceaim = 1
#                 else:
#                     normalized_value = resultantforce / 10
#                     replaceaim = np.sqrt(normalized_value)
        
#         return replaceaim
    