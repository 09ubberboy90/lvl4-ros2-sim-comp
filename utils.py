import math

def convert_to_euler(pose_mat):
    yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
    pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
    roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,yaw,pitch,roll]

def convert_to_quaternion(pose_mat):
    # Per issue #2, adding a abs() so that sqrt only results in real numbers
    r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
    r_x = (pose_mat[2][1]-pose_mat[1][2])/(4*r_w)
    r_y = (pose_mat[0][2]-pose_mat[2][0])/(4*r_w)
    r_z = (pose_mat[1][0]-pose_mat[0][1])/(4*r_w)

    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [x,y,z,r_w,r_x,r_y,r_z]