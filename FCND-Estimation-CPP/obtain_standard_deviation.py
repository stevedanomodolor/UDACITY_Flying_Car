import numpy as np

def obtain_file_data(filename):
    with open(filename, "r") as file:
        data = file.readlines()
    return data
def get_data_name(filename):
    data = obtain_file_data(filename)
    return data[0].strip().split()
def get_data(filename):
    data = obtain_file_data(filename)
    data = data[1:]
    data = [line.strip().split() for line in data]
    new_data = []
    for row in data:
        new_data.append([float(i) for i in row[0].split(",")])
    data = np.array(new_data)
    return data

if __name__ == "__main__":
    gps_filename = "/home/stevedan/UDACITY_Flying_Car/FCND-Estimation-CPP/config/log/Graph1.txt"  
    accel_filename = "/home/stevedan/UDACITY_Flying_Car/FCND-Estimation-CPP/config/log/Graph2.txt"  
    gps_data = get_data(gps_filename)
    accel_data = get_data(accel_filename)

    std_deviation_gps = np.std(gps_data[:,1], axis=0)
    std_deviation_accel = np.std(accel_data[:,1], axis=0)
    print("Standard deviation gps x: ", std_deviation_gps)
    print("Standard deviation accel x: ",std_deviation_accel)
