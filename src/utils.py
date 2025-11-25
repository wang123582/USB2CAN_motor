# utils.py
# 处理达妙电机 MIT 模式所需的定点数与浮点数转换
def float_to_uint(x, x_min, x_max, bits):
    """
    将浮点数转换为无符号整数 (用于达妙 MIT 协议发送)
    """
    span = x_max - x_min
    offset = x_min
    
    # 限制范围
    if x > x_max: x = x_max
    if x < x_min: x = x_min
    
    return int((x - offset) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    """
    将无符号整数转换为浮点数 (用于达妙 MIT 协议接收)
    """
    span = x_max - x_min
    offset = x_min
    return float(x_int) * span / ((1 << bits) - 1) + offset

def clip(val, min_val, max_val):
    """通用限幅"""
    return max(min(val, max_val), min_val)