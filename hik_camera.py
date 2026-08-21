import sys
import cv2
from ctypes import *

if sys.platform.startswith("win"):
    sys.path.append("./MvImport")
    from MvImport.MvCameraControl_class import *
else:
    sys.path.append("./MvImport_Linux")
    from MvImport_Linux.MvCameraControl_class import *


def _hik_serial_from_ubyte_array(buf):
    s = ""
    for per in buf:
        if per == 0:
            break
        s += chr(per)
    return s.strip()


def hik_device_serial(mvcc_dev_info):
    """从 MV_CC_DEVICE_INFO 解析序列号（GigE / USB3 等）。"""
    if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
        return _hik_serial_from_ubyte_array(mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber)
    if mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
        return _hik_serial_from_ubyte_array(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
    return ""


def select_hik_device_index(deviceList, serial_wanted=None, default_index=0):
    """
    在 MV_CC_EnumDevices 结果中选择设备下标。
    serial_wanted 非空时按序列号匹配（不区分大小写）；为空则使用 default_index。
    指定了序列号但未匹配到任何设备时退出进程并打印已枚举设备的序列号。
    """
    n = int(deviceList.nDeviceNum)
    if n <= 0:
        return 0
    want = (serial_wanted or "").strip()
    if not want:
        idx = int(default_index)
        if idx < 0 or idx >= n:
            print("设备索引越界: index=%s, devices=%s" % (idx, n))
            sys.exit(1)
        return idx
    want_l = want.lower()
    for i in range(n):
        mv = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        sn = hik_device_serial(mv)
        if sn.lower() == want_l:
            print("已选择海康设备: 枚举索引=%d, 序列号=%s" % (i, sn))
            return i
    print("错误: 未找到序列号为 %r 的海康设备。当前枚举到的序列号:" % want)
    for i in range(n):
        mv = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        print("  [%d] %s" % (i, hik_device_serial(mv) or "(无/未知类型)"))
    sys.exit(1)


# 将 SDK 原始帧数据转换为 OpenCV BGR 图像
def image_control(data, stFrameInfo):
    image = None
    if stFrameInfo.enPixelType == 17301505:  # Mono8
        image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
    elif stFrameInfo.enPixelType == 17301513:  # BayerRG8
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_BAYER_RG2RGB)
    elif stFrameInfo.enPixelType == 35127316:  # RGB8
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    elif stFrameInfo.enPixelType == 34603039:  # YUV422
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
    return image


# 获取各种类型节点参数
def get_Value(cam, param_type="int_value", node_name="PayloadSize"):
    """
    :param cam:            相机实例
    :param param_type:     节点值类型：int/float/enum/bool/string
    :param node_name:      节点名
    :return:               节点值
    """
    if param_type == "int_value":
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = cam.MV_CC_GetIntValueEx(node_name, stParam)
        if ret != 0:
            print("获取 int 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        int_value = stParam.nCurValue
        return int_value

    elif param_type == "float_value":
        stFloatValue = MVCC_FLOATVALUE()
        memset(byref(stFloatValue), 0, sizeof(MVCC_FLOATVALUE))
        ret = cam.MV_CC_GetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("获取 float 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        float_value = stFloatValue.fCurValue
        return float_value

    elif param_type == "enum_value":
        stEnumValue = MVCC_ENUMVALUE()
        memset(byref(stEnumValue), 0, sizeof(MVCC_ENUMVALUE))
        ret = cam.MV_CC_GetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("获取 enum 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        enum_value = stEnumValue.nCurValue
        return enum_value

    elif param_type == "bool_value":
        stBool = c_bool(False)
        ret = cam.MV_CC_GetBoolValue(node_name, stBool)
        if ret != 0:
            print("获取 bool 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        return stBool.value

    elif param_type == "string_value":
        stStringValue = MVCC_STRINGVALUE()
        memset(byref(stStringValue), 0, sizeof(MVCC_STRINGVALUE))
        ret = cam.MV_CC_GetStringValue(node_name, stStringValue)
        if ret != 0:
            print("获取 string 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        string_value = stStringValue.chCurValue
        return string_value


# 设置各种类型节点参数
def set_Value(cam, param_type="int_value", node_name="PayloadSize", node_value=None):
    """
    :param cam:               相机实例
    :param param_type:        需要设置的节点值类型
        int/float:            数值
        enum:                 参考于客户端中该选项的 Enum Entry Value 值
        bool:                 0 为关，1 为开
        string:               数字或英文字符，不能为汉字
    :param node_name:         需要设置的节点名
    :param node_value:        设置给节点的值
    """
    if param_type == "int_value":
        stParam = int(node_value)
        ret = cam.MV_CC_SetIntValueEx(node_name, stParam)
        if ret != 0:
            print("设置 int 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 int 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "float_value":
        stFloatValue = float(node_value)
        ret = cam.MV_CC_SetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("设置 float 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 float 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "enum_value":
        stEnumValue = node_value
        ret = cam.MV_CC_SetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("设置 enum 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 enum 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "bool_value":
        ret = cam.MV_CC_SetBoolValue(node_name, node_value)
        if ret != 0:
            print("设置 bool 型数据节点 %s 失败 ！ 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 bool 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "string_value":
        stStringValue = str(node_value)
        ret = cam.MV_CC_SetStringValue(node_name, stStringValue)
        if ret != 0:
            print("设置 string 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 string 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))


# 开启取流
def start_grab_and_get_data_size(cam):
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("开始取流失败! ret[0x%x]" % ret)
        sys.exit()
