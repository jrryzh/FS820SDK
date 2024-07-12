import OpenEXR
import Imath
import numpy as np
import cv2

def save_array_to_png(depth_array: np.array, 
                      file_path: str,
                      max_z: float):
    """将numpy array转化为png格式的图像文件，depth_array和max_z的单位必须一致。
    Args:
        depth_array(np.array): 深度信息矩阵
        file_path(str): png格式的深度图保存文件路径
        max_z(float): 最大深度，默认单位为毫米（mm）
    """
    cv2.imwrite(file_path, (depth_array/max_z*65535).astype(np.uint16))


def read_png_to_array(file_path:str, 
                      max_z:float):
    """将EXR格式的深度图文件转化为numpy array
    Args:
        file_path(str): EXR格式的深度图文件路径
    """
    img_depth = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
    img_depth = img_depth.astype(np.float64) / 65535 * max_z
    

def save_array_to_exr(file_path: str,
                      depth_array: np.array):
    """将numpy array转化为exr格式的图像文件
    Args:
        depth_array(np.array): 深度信息矩阵
        file_path(str): EXR格式的深度图保存文件路径
    """
    # Ensure the depth array is of type float32
    depth_array = np.array(depth_array, dtype=np.float32)
    height, width = depth_array.shape

    # Create an OpenEXR header with the appropriate channel configuration
    header = OpenEXR.Header(width, height)
    header['channels'] = {'Z': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT))}

    # Create the OpenEXR image object
    exr_image = OpenEXR.OutputFile(file_path, header)
    exr_image.writePixels({'Z': depth_array.tobytes()})
    

def read_exr_to_array(file_path:str):
    """将EXR格式的深度图文件转化为numpy array
    Args:
        file_path(str): EXR格式的深度图文件路径
    """
    
    # Open the EXR file
    input_file = OpenEXR.InputFile(file_path)

    # Read the header to get the image dimensions and channel info
    dw = input_file.header()['dataWindow']
    width, height = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    channels = input_file.header()['channels']

    # Check if the 'Z' channel exists in the file
    if 'Z' not in channels:
        raise ValueError("The 'Z' channel is not present in the EXR file.")

    # Read the 'Z' channel data from the EXR file
    z_str = input_file.channel('Z', Imath.PixelType(Imath.PixelType.FLOAT))
    z_data = np.frombuffer(z_str, dtype=np.float32)
    z_data = z_data.reshape(height, width)

    # Close the file
    input_file.close()
    return z_data