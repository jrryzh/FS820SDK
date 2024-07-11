import pcammls
from pcammls import * 
import cv2
import numpy
import sys
import os

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
        
        color_fmt_list = self.cl.DeviceStreamFormatDump(self.handle, PERCIPIO_STREAM_COLOR)
        if len(color_fmt_list) == 0:
            print ('device has no color stream.')
            return

        print ('color image format list:')
        for idx in range(len(color_fmt_list)):
            fmt = color_fmt_list[idx]
            print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, self.cl.Width(fmt), self.cl.Height(fmt), fmt.getDesc()))
        self.cl.DeviceStreamFormatConfig(self.handle, PERCIPIO_STREAM_COLOR, color_fmt_list[0])

        depth_fmt_list = self.cl.DeviceStreamFormatDump(self.handle, PERCIPIO_STREAM_DEPTH)
        if len(depth_fmt_list) == 0:
            print ('device has no depth stream.')
            return

        print ('depth image format list:')
        for idx in range(len(depth_fmt_list)):
            fmt = depth_fmt_list[idx]
            print ('\t{} -size[{}x{}]\t-\t desc:{}'.format(idx, self.cl.Width(fmt), self.cl.Height(fmt), fmt.getDesc()))
        self.cl.DeviceStreamFormatConfig(self.handle, PERCIPIO_STREAM_DEPTH, depth_fmt_list[0])

        scale_unit = self.cl.DeviceReadCalibDepthScaleUnit(self.handle)
        print ('depth image scale unit :{}'.format(scale_unit))

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

        
        self.cl.DeviceStreamMapDepthImageToColorCoordinate(depth_calib.data(), img_depth.width, img_depth.height, scale_unit,  img_depth,  color_calib.data(), img_color.width, img_color.height, img_registration_depth)
        
        self.cl.DeviceStreamDepthRender(img_registration_depth, img_registration_render)
        mat_depth_render = img_registration_render.as_nparray()
        cv2.imshow('registration', mat_depth_render)

        self.cl.DeviceStreamImageDecode(img_color, img_parsed_color)
        self.cl.DeviceStreamDoUndistortion(color_calib.data(), img_parsed_color, img_undistortion_color)
        mat_undistortion_color = img_undistortion_color.as_nparray()
        cv2.imshow('undistortion rgb', mat_undistortion_color)
        
        
    def get_hdr_by_targetlights(self, depth_path, gray_path, TargetLights=[40, 50, 60]):
                
        pass
    
    
if __name__ == '__main__':
    interface = FS820SDKInterface()
    intrinsic = interface.get_camera_intrinsic()
    print(intrinsic)
    iterface.get_image_gray_and_depth(depth_path="./depth.png", gray_path="./gray.png", lr_gray_path="./lr_gray.png", exposure_time=80.0, isOpen=True, TargetLight=50)