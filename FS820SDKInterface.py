import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os
import numpy as np
import time

from camera_util import save_array_to_exr, read_exr_to_array

class PythonPercipioDeviceEvent(pcammls.DeviceEvent):
    Offline = False

    def __init__(self):
        pcammls.DeviceEvent.__init__(self)

    def run(self, handle, eventID):
        if eventID==TY_EVENT_DEVICE_OFFLINE:
          print('=== Event Callback: Device Offline!')
          self.Offline = True
        return 0

    def IsOffline(self):
        return self.Offline

class FS820SDKInterface:
    
    def __init__(self):
        """连接相机，显示当前内参
        """
        self.cl = PercipioSDK()

        self.dev_list = self.cl.ListDevice()
        for idx in range(len(self.dev_list)):
            dev = self.dev_list[idx]
            print ('{} -- {} \t {}'.format(idx,dev.id,dev.iface.id))
        if  len(self.dev_list)==0:
            print ('no device')
            return
        if len(self.dev_list) == 1:
            selected_idx = 0 
        else:
            selected_idx  = int(input('select a device:'))
        if selected_idx < 0 or selected_idx >= len(self.dev_list):
            return

        self.sn = self.dev_list[selected_idx].id

        self.handle = self.cl.Open(self.sn)
        if not self.cl.isValidHandle(self.handle):
            print('no device found')
            return
        
        self.event = PythonPercipioDeviceEvent()
        self.cl.DeviceRegiststerCallBackEvent(self.event)
        
        # 将当前rgb和depth流打开
        self.cl.DeviceStreamEnable(self.handle, PERCIPIO_STREAM_COLOR | PERCIPIO_STREAM_DEPTH)
    
    def get_camera_intrinsic(self):
        """
        Returns:
            instrinsic(np.array): 相机内参矩阵(3X3)，单位mm
            >>>[[fx, 0, cx],
                [0, fy, cy],
                [0,  0,  1]]
        """
        color_calib_data = self.cl.DeviceReadCalibData(self.handle, PERCIPIO_STREAM_COLOR)
        color_calib_width = color_calib_data.Width()
        color_calib_height = color_calib_data.Height()
        color_calib_intr = color_calib_data.Intrinsic() # CalibDataVector
        print("Camera instrinsic: \n{}".format(color_calib_intr))
        return color_calib_intr
    
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
        
        ###################### 初始化开始 ######################
        
        # 查看当前曝光时间
        value = TYGetInt(self.handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME) 
        print (f"Current exposure_time: {value}")
        
        # 查看范围
        # 实例化对象
        EL = TY_INT_RANGE() 
        # #获取曝光调节范围 
        TYGetIntRange(self.handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, EL) 
        print(EL.CSize())
        #打印最小值 最大值 
        print(EL.min,EL.max,EL.inc)
        
        # 设置曝光时间
        exposure_time = int(exposure_time)
        TYSetInt(self.handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, exposure_time)
        
        # 查看当前曝光时间
        value = TYGetInt(self.handle, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME) 
        print (f" NEW exposure_time: {value}")
        
        # 设置激光强度
        TYSetInt(self.handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, TargetLight)
        
        # 查看当前激光强度
        Value = TYGetInt(self.handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER)
        print (f"Current Laser Power: {Value}")
        
        
        ###################### 初始化结束 ######################
        
        # 初始化rgb流格式
        color_fmt_list = self.cl.DeviceStreamFormatDump(self.handle, PERCIPIO_STREAM_COLOR)
        if len(color_fmt_list) == 0:
            print ('device has no color stream.')
            return

        print ('color image format list:')
        for idx in range(len(color_fmt_list)):
            fmt = color_fmt_list[idx]
            print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, self.cl.Width(fmt), self.cl.Height(fmt), fmt.getDesc()))
        self.cl.DeviceStreamFormatConfig(self.handle, PERCIPIO_STREAM_COLOR, color_fmt_list[0])
        
        # 初始化depth流格式
        depth_fmt_list = self.cl.DeviceStreamFormatDump(self.handle, PERCIPIO_STREAM_DEPTH)
        if len(depth_fmt_list) == 0:
            print ('device has no depth stream.')
            return

        print ('depth image format list:')
        for idx in range(len(depth_fmt_list)):
            fmt = depth_fmt_list[idx]
            print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, self.cl.Width(fmt), self.cl.Height(fmt), fmt.getDesc()))
        self.cl.DeviceStreamFormatConfig(self.handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

        # 设置scale_unit
        scale_unit = self.cl.DeviceReadCalibDepthScaleUnit(self.handle)
        print ('depth image scale unit :{}'.format(scale_unit))
        
        # 打开rgb和depth流
        depth_calib = self.cl.DeviceReadCalibData(self.handle, PERCIPIO_STREAM_DEPTH)
        color_calib = self.cl.DeviceReadCalibData(self.handle, PERCIPIO_STREAM_COLOR)

        self.cl.DeviceStreamOn(self.handle)
        img_registration_depth  = image_data()
        img_registration_render = image_data()
        img_parsed_color        = image_data()
        img_undistortion_color  = image_data()
        
        image_list = self.cl.DeviceStreamRead(self.handle, 2000)
        if len(image_list) == 2:
            for i in range(len(image_list)):
                frame = image_list[i]
                if frame.streamID == PERCIPIO_STREAM_DEPTH:
                    img_depth = frame
                if frame.streamID == PERCIPIO_STREAM_COLOR:
                    img_color = frame

        # 将depth对齐到rgb
        self.cl.DeviceStreamMapDepthImageToColorCoordinate(depth_calib.data(), img_depth.width, img_depth.height, scale_unit,  img_depth,  color_calib.data(), img_color.width, img_color.height, img_registration_depth)    
        
        # 保存depth图
        depth_img_path = depth_path.split('.')[0] + '.png'
        self.cl.DeviceStreamDepthRender(img_registration_depth, img_registration_render)
        mat_depth_render = img_registration_render.as_nparray()
        cv2.imwrite(depth_img_path, mat_depth_render)
        
        # 保存depth exr 后面使用
        mat_depth_registration = img_registration_depth.as_nparray()
        max_z = 1300.0
        depth_exr = mat_depth_registration.astype(np.float32) / 65535.0 * max_z
        depth_exr = depth_exr.squeeze()
        save_array_to_exr(depth_path, depth_exr)
        
        # 保存rgb图
        self.cl.DeviceStreamImageDecode(img_color, img_parsed_color)
        self.cl.DeviceStreamDoUndistortion(color_calib.data(), img_parsed_color, img_undistortion_color)
        mat_undistortion_color = img_undistortion_color.as_nparray()
        cv2.imwrite(gray_path, mat_undistortion_color)
        
        return 0
        
        
    def get_hdr_by_targetlights(self, depth_path, gray_path, TargetLights=[40, 50, 60]):
        """
        hdr by multiple target lights
        """
        # hdr by target lights
        depth_list = []
        for i, TargetLight in enumerate(TargetLights):
            tmp_lr_gray_path = '/home/yofo/fs820/testoutput/tmp/lr_gray_{}.png'.format(TargetLight)
            tmp_depth_path = '/home/yofo/fs820/testoutput/tmp/depth_{}.exr'.format(TargetLight)
            tmp_gray_path = '/home/yofo/fs820/testoutput/tmp/gray_{}.png'.format(TargetLight)
            if i == len(TargetLights)-1:
                tmp_gray_path = gray_path

            self.get_image_gray_and_depth(tmp_depth_path, tmp_gray_path, lr_gray_path=tmp_lr_gray_path, exposure_time=80.0,
                                     isOpen=True, TargetLight=TargetLight)
            time.sleep(10)
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
    interface = FS820SDKInterface()
    intrinsic = interface.get_camera_intrinsic()
    interface.get_image_gray_and_depth(depth_path="/home/yofo/fs820/testoutput/depth.exr", gray_path="/home/yofo/fs820/testoutput/gray.png", lr_gray_path="/home/yofo/fs820/testoutput/lr_gray.png", exposure_time=80.0, isOpen=True, TargetLight=50)
    # interface.get_hdr_by_targetlights(depth_path="/home/yofo/fs820/testoutput/depth_hdr.exr", gray_path="/home/yofo/fs820/testoutput/gray_hdr.png", TargetLights=[40, 50, 60])