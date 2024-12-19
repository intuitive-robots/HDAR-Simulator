import numpy as np
import os
import pickle
import matplotlib.pyplot as plt

#path of the files
hdar_path = os.path.dirname(os.path.abspath(__file__))
print(hdar_path)
data_path = os.path.join(hdar_path, "SFDemoData/", "PickandPlaceBox_2024_11_07_18_30_29/")
print(data_path)
files_list = os.listdir(data_path)
results = []
for file in files_list:
    file_path = os.path.join(data_path, file)
    with open(file_path, "rb") as f:
       recorded_data = pickle.load(f)
    state_data = recorded_data["state"]
    robot_data = state_data.get("panda_robot")
    picked_box_data = state_data.get("picked_box")
    target_box_data = state_data.get("target_box")
    if robot_data is None:
        raise KeyError("'panda_robot' key not found in 'state' data. Available keys: {}".format(state_data.keys()))

# Extract relevant recorded data arrays
    j_pos_data = robot_data["des_j_pos"]
    current_c_pos = robot_data["c_pos"]
    j_pos_acc = robot_data["des_j_acc"]
    gripper_width_data = robot_data["gripper_width"]
    picked_box_pos = picked_box_data["pos"]
    picked_box_quat = picked_box_data["quat"]

    target_box_pos = target_box_data["pos"]
    target_box_quat = target_box_data["quat"]
    tcp_position = np.array(current_c_pos)
    box_position = np.array(picked_box_pos)
    
#tcp norm length
    tcp_actual_length = 0
    for j in range(1,len(tcp_position)):
        distance = np.linalg.norm(tcp_position[j]-tcp_position[j-1])
        tcp_actual_length += distance
    
    start_position = tcp_position[0]
    end_position = tcp_position[-1:] 
    straight_length = np.linalg.norm(end_position-start_position)
    normal_distance = tcp_actual_length / straight_length 
    print(f"norm length :{normal_distance}")
#tcp jerk RMS
    velocities = np.diff(tcp_position, axis=0) / 0.1 
    accelerations = np.diff(velocities, axis=0) / 0.1
    jerks = np.diff(accelerations, axis=0) / 0.1
    rms_jerk = np.sqrt(np.mean(np.sum(jerks ** 2, axis=1)))
    print(f"jerk RMS: {rms_jerk:.4f}")

#box norm length
    box_actual_length = 0
    for k in range(1,len(box_position)):
        box_distance = np.linalg.norm(box_position[k]-box_position[k-1])
        box_actual_length += box_distance
    
    box_start_position = box_position[0]
    box_end_position = box_position[-1:] 
    box_straight_length = np.linalg.norm(box_end_position-box_start_position)
    box_normal_distance = box_actual_length / box_straight_length 
    print(f"box norm length :{box_normal_distance}")

    results.append({
        'file_name': file,
        'tcp_jerk': rms_jerk,
        'tcp_normalized_trajectory_length': normal_distance,
        'box_normalized_trajectory_length': box_normal_distance,
    })


avg_normalized_length = np.mean([result['box_normalized_trajectory_length'] for result in results])
avg_RMS = np.mean([result['tcp_jerk'] for result in results])
variance_normalized_length = np.var([result['box_normalized_trajectory_length'] for result in results])
std_dev = np.sqrt(variance_normalized_length)
print(f"avarage rms:{avg_RMS:.4f},avarage box norm length: {avg_normalized_length:.4f},box norm length variance:{variance_normalized_length:.4f}")



plt.figure(figsize=(5, 10))
box_lengths = [result['box_normalized_trajectory_length'] for result in results]
plt.scatter(range(len(box_lengths)), box_lengths, color='blue', label='Box Normalized Lengths')

plt.axhline(y=avg_normalized_length, color='red', linestyle='-', linewidth=3, label=f'Average ({avg_normalized_length:.4f})')
plt.fill_between(
    range(len(box_lengths)),
    avg_normalized_length - std_dev,
    avg_normalized_length + std_dev,
    color='red',
    alpha=0.15,
    # label=f'±1 Standard Deviation ({std_dev:.4f})'
)

plt.xlabel('Sample Index')
plt.ylabel('Box Normalized Trajectory Length')
plt.title('Box Normalized Trajectory Lengths with Average and Variance')
plt.legend()
plt.grid(True)
plt.show()