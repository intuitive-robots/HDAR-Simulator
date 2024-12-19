import os

#the path of files
hdar_path = os.path.dirname(os.path.abspath(__file__))
print(hdar_path)
folder_path = os.path.join(hdar_path, "SFDemoData/", "PickandPlaceBox_2024_11_08_14_49_35/")
print(folder_path)



files = sorted([f for f in os.listdir(folder_path) if f.endswith(".pkl")])

# start name's number 
start_index = 212

# rename
for i, file_name in enumerate(files):
    # get the name before the number
    prefix = file_name.split('_')[0]
    
    # Create the new name with the incremented index
    new_name = f"{prefix}_{str(start_index + i).zfill(3)}.pkl"
    
    # Full paths for old and new files
    old_file = os.path.join(folder_path, file_name)
    new_file = os.path.join(folder_path, new_name)
    
    # Rename the file
    os.rename(old_file, new_file)

    print(f"Renamed: {file_name} -> {new_name}")
