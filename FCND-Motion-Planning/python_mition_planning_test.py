import numpy as np

def obtain_lat0_lon0(data):
    with open(data) as f:
        first_row = f.readline().strip('\n')
        pose_ = first_row.split(", ")
        lat0 = float(pose_[0].split(" ")[1])
        lon0 = float(pose_[1].split(" ")[1])
    return lat0, lon0
if __name__ == "__main__":


    # with open('colliders.csv') as f:
    #     first_line = f.readline().strip('\n')
    #     x = first_line.split(", ")
    #     for i in range(len(x)):
    #         home_pose.append(float(x[i].split(" ")[1]))
    print(obtain_lat0_lon0('colliders.csv'))
