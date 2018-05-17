#import the necessary modules
import freenect
import cv2
import numpy as np
import os
from py3d import *
import numpy as np
import matplotlib.pyplot as plt

 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

if __name__ == "__main__":
    cntr = 0
    pinhole_camera_intrinsic = PinholeCameraIntrinsic(PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    while 1:
        cntr+=1
        frame = get_video()
        depth = get_depth()
        current_directory = os.getcwd()
        path_f = os.path.join(current_directory, r'rgb')
        if not os.path.exists(path_f):
            os.makedirs(path_f)
        path_f = os.path.join(current_directory, r'depth')
        if not os.path.exists(path_d):
            os.makedirs(path_d)
        cv2.imwrite(os.path.join(path_f , str(cntr)+'.jpg'), frame)
        cv2.imwrite(os.path.join(path_d , str(cntr)+'.jpg'), depth)
        if cntr>1:
            source_color = read_image("rgb/"+str(cntr-1)+".jpg")
            source_depth = read_image("depth/"+str(cntr-1)+".jpg")
            target_color = read_image("rgb/"+str(cntr)+".jpg")
            target_depth = read_image("depth/"+str(cntr)+".jpg")
            source_rgbd_image = create_rgbd_image_from_color_and_depth(source_color, source_depth)
            target_rgbd_image = create_rgbd_image_from_color_and_depth(target_color, target_depth)
            target_pcd = create_point_cloud_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)

            option = OdometryOption()
            odo_init = np.identity(4)
            print(option)

            #[success_color_term, trans_color_term, info] = compute_rgbd_odometry(source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic, odo_init,RGBDOdometryJacobianFromColorTerm(), option)
            [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(source_rgbd_image, target_rgbd_image, pinhole_camera_intrinsic, odo_init, RGBDOdometryJacobianFromHybridTerm(), option)
            #if success_color_term:
                #print("Using RGB-D Odometry")
                #print(trans_color_term)
                #source_pcd_color_term = create_point_cloud_from_rgbd_image(source_rgbd_image, pinhole_camera_intrinsic)
                #source_pcd_color_term.transform(trans_color_term)
                #draw_geometries([target_pcd, source_pcd_color_term])
            if success_hybrid_term:
                #print("Using Hybrid RGB-D Odometry")
                #print(trans_hybrid_term)
                source_pcd_hybrid_term = create_point_cloud_from_rgbd_image(source_rgbd_image, pinhole_camera_intrinsic)
                source_pcd_hybrid_term.transform(trans_hybrid_term)
                #draw_geometries([target_pcd, source_pcd_hybrid_term])
                ax.scatter3D(trans_hybrid_term[0][3], trans_hybrid_term[1][3], trans_hybrid_term[2][3], c=trans_hybrid_term[2][3], cmap='Greens')
                plt.pause(0.5)

        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    plt.show()
    cv2.destroyAllWindows()
