import sys
import os
import numpy as np
import VSensorSDK
import cv2
from VSensorSDK import VSensorCameraInternelPara

parent_parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, parent_parent_dir)
from camera_util import save_array_to_exr, read_exr_to_array

class VSensorSDKInterface:

    def __init__(self):
        """连接相机，显示当前内参
        """
        DevList = VSensorSDK.GetDeviceList()
        err_code = VSensorSDK.GetLastError()
        if err_code != 0:
            print("get device list failed, status = {}".format(err_code))
            return
        else:
            print("get device list success, device num  = {}".format(len(DevList)))
        for i, DevInfo in enumerate(DevList):
            print("device num {}: name {}".format(i, DevInfo.GetCameraName()))

        err_code = VSensorSDK.DeviceConnect(0)
        if err_code != 0:
            print("connect device failed status = {}".format(err_code))
            return
        else:
            print("connect device success")
        camera_info = VSensorSDK.VSensorCameraInternelPara()
        info = VSensorSDK.GetCamInternelParameter(camera_info)
        print("Left Camera intrinsic parameters\n", np.array(camera_info.LPara))



    def get_camera_intrinsic(self):
        """
        Returns:
            instrinsic(np.array): 相机内参矩阵(3X3)，单位mm
            >>>[[fx, 0, cx],
                [0, fy, cy],
                [0,  0,  1]]
        """
        cam_info = VSensorCameraInternelPara()
        info = VSensorSDK.GetCamInternelParameter(cam_info)
        cam_info = np.array(cam_info.LPara)
        fx = cam_info[5]
        fy = cam_info[6]
        cx = cam_info[7]
        cy = cam_info[8]
        intrinsic_array = np.array(
            [
                [fx, 0, cx],
                [0, fy, cy],
                [0,  0,  1]
            ]
        )
        print('intrinsic_array')
        print(intrinsic_array)
        return intrinsic_array



    def get_image_gray_and_depth(self, 
                                 depth_path: str, 
                                 gray_path: str, 
                                 lr_gray_path: str = None,
                                 exposure_time: float =80.0, 
                                 isOpen: bool =True, 
                                 TargetLight: int =50):
        """
        Args:
            depth_path: 深度图保存路径
            gray_depth: 灰度图保存路径
            isOpen: 自动曝光模型开关(False 关闭，True 开启)
            exposure_time: 自动曝光模式关闭时起作用，曝光时间
            TargetLight: 自动曝光模式开启时起作用，目标光量
        Returns:
            ErrorCode(int): 0-正常，其他-错误
        """
        if lr_gray_path is None:
            lr_gray_path = '/tmp/lr_gray.png'

        # 自动曝光
        err_code = VSensorSDK.SetAutoExposureTime(isOpen=isOpen, TargetLight=TargetLight)
        if err_code != 0:
            print("set SetAutoExposureTime failed status = {}".format(err_code))
            return err_code
        else:
            print("set SetAutoExposureTime success")

        # 设置曝光时间()
        err_code = VSensorSDK.SetExposureTime(exposure_time)
        if err_code != 0:
            print("set exposuretime {} failed status = {}".format(exposure_time, err_code))
            return err_code
        else:
            print("set exposuretime {} success".format(exposure_time))

        # 设置为采集模式
        err_code = VSensorSDK.SetCaptureMode(VSensorSDK.MODE_CAPTURE)
        if err_code != 0:
            print("set capture mode failed status = {}".format(err_code))
            return err_code
        else:
            print("set capture mode success")

        # 单次采集测试
        print("Start single capture...")
        result = VSensorSDK.VSensorResult()
        result = VSensorSDK.SingleRestruction(result, VSensorSDK.OUTPUT_MODE_ALL)
        err_code = VSensorSDK.GetLastError()
        if err_code != 0:
            print("single resturction failed status = {}".format(err_code))
            return err_code
        else:
            print("single resturction success")

        # 保存左右两侧的灰度图
        err_code = VSensorSDK.SaveGrayMap(lr_gray_path, result, VSensorSDK.GRAY_TYPR_CORRECT)
        lr_gray = cv2.imread(lr_gray_path)
        left_half_width = lr_gray.shape[1] // 2
        left_half = lr_gray[:, :left_half_width]
        
        # 保存左侧灰度图
        cv2.imwrite(gray_path, left_half)
        if err_code != 0:
            print("save gray image failed status = {}".format(err_code))
            return err_code
        else:
            print("save gray image success")
            
        # 保存深度图
        cached_depth_path = '/tmp/depth.png'
        err_code = VSensorSDK.SaveDepthMap(cached_depth_path, result)
        if err_code != 0:
            print("save depth image failed status = {}".format(err_code))
            return err_code
        else:
            print("save depth image success")
        img_depth = cv2.imread(cached_depth_path, cv2.IMREAD_UNCHANGED)
        max_z = 1300.0
        img_depth = img_depth.astype(np.float32) / 65535 * max_z
        save_array_to_exr(depth_path, img_depth)
        
        depth_png_path = f"{os.path.splitext(depth_path)[0]}.png"
        cv2.imwrite(depth_png_path, img_depth)
        
        return 0

    def get_hdr_by_targetlights(self, depth_path, gray_path, TargetLights=[40, 50, 60]):
        """
        hdr by multiple target lights
        """
        # hdr by target lights
        depth_list = []
        for i, TargetLight in enumerate(TargetLights):
            tmp_lr_gray_path = '/tmp/lr_gray_{}.png'.format(TargetLight)
            tmp_depth_path = '/tmp/depth_{}.exr'.format(TargetLight)
            tmp_gray_path = '/tmp/gray_{}.png'.format(TargetLight)
            if i == len(TargetLights)-1:
                tmp_gray_path = gray_path

            self.get_image_gray_and_depth(tmp_depth_path, tmp_gray_path, lr_gray_path=tmp_lr_gray_path, exposure_time=80.0,
                                     isOpen=True, TargetLight=TargetLight)
            depth_list.append(read_exr_to_array(tmp_depth_path))

        mask_list = [(c!=0).astype(int) for c in depth_list]
        num_contributing_images = sum(mask_list)
        sum_values = sum([c.astype(float) for c in depth_list])
        print('sum_values')
        print(sum_values.dtype)
        average_image = np.divide(sum_values, num_contributing_images.astype(float), out=np.zeros_like(sum_values),
                                  where=num_contributing_images != 0)
        save_array_to_exr(depth_path, average_image.astype(np.uint16))



if __name__ == '__main__':
    vsensor = VSensorSDKInterface()
    vsensor.get_image_gray_and_depth('depth.exr', 'gray.png', 'lr_gray.png', exposure_time=80.0, isOpen=True, TargetLight=50)

    # vsensor.get_hdr_by_targetlights('depth.exr', 'gray.png', lr_gray_path=None, exposure_time=80.0,
                            # TargetLights=[40, 50, 60])
    

