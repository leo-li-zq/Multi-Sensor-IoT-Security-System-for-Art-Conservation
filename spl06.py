# spl06.py (V6/V9 驱动 - 会产生-1782hPa跳变, 但不会死机)
import time
import smbus2

# --- 寄存器地址 ---
REG_PRS_B2 = 0x00
REG_PRS_B1 = 0x01
REG_PRS_B0 = 0x02
REG_TMP_B2 = 0x03
REG_TMP_B1 = 0x04
REG_TMP_B0 = 0x05
REG_PRS_CFG = 0x06
REG_TMP_CFG = 0x07
REG_MEAS_CFG = 0x08
REG_CFG = 0x09
REG_RESET = 0x0C
REG_ID = 0x0D
REG_COEF_START = 0x10


# 补偿系数
class Compensation:
    def __init__(self):
        self.c0 = 0
        self.c1 = 0
        self.c00 = 0
        self.c10 = 0
        self.c01 = 0
        self.c11 = 0
        self.c20 = 0
        self.c21 = 0
        self.c30 = 0


class SPL06:
    def __init__(self, bus, address=0x76):
        self.bus = bus
        self.addr = address
        self.comp = Compensation()

        if self.bus.read_byte_data(self.addr, REG_ID) != 0x10:
            raise RuntimeError("SPL06: 未找到芯片, ID不匹配 (地址 0x76)")

        self._get_compensation()
        self._set_sampling_rates()
        print("SPL06 (气压) 在 0x76 初始化成功。 (V6/V9 驱动 - 连续模式)")

    def _twos_complement(self, val, bits):
        """计算补码"""
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val

    def _get_compensation(self):
        """读取18字节的补偿系数"""
        data = self.bus.read_i2c_block_data(self.addr, REG_COEF_START, 18)

        self.comp.c0 = (data[0] << 4) | (data[1] >> 4)
        self.comp.c0 = self._twos_complement(self.comp.c0, 12)

        self.comp.c1 = ((data[1] & 0x0F) << 8) | data[2]
        self.comp.c1 = self._twos_complement(self.comp.c1, 12)

        self.comp.c00 = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        self.comp.c00 = self._twos_complement(self.comp.c00, 20)

        self.comp.c10 = ((data[5] & 0x0F) << 16) | (data[6] << 8) | data[7]
        self.comp.c10 = self._twos_complement(self.comp.c10, 20)

        self.comp.c01 = (data[8] << 8) | data[9]
        self.comp.c01 = self._twos_complement(self.comp.c01, 16)

        self.comp.c11 = (data[10] << 8) | data[11]
        self.comp.c11 = self._twos_complement(self.comp.c11, 16)

        self.comp.c20 = (data[12] << 8) | data[13]
        self.comp.c20 = self._twos_complement(self.comp.c20, 16)

        self.comp.c21 = (data[14] << 8) | data[15]
        self.comp.c21 = self._twos_complement(self.comp.c21, 16)

        self.comp.c30 = (data[16] << 8) | data[17]
        self.comp.c30 = self._twos_complement(self.comp.c30, 16)

    def _set_sampling_rates(self):
        """设置采样率 (16x) 并进入连续模式"""
        self.bus.write_byte_data(self.addr, REG_PRS_CFG, 0x24)
        self.bus.write_byte_data(self.addr, REG_TMP_CFG, 0xA4)
        self.bus.write_byte_data(self.addr, REG_CFG, 0x06)
        self.bus.write_byte_data(self.addr, REG_MEAS_CFG, 0x07)  # 连续模式
        time.sleep(0.1)

    def get_pressure(self):
        """获取气压值 (hPa) - V6/V9 驱动 (会产生跳变)"""
        k_T = 524288.0
        k_P = 1572864.0  # <--- 使用这个错误的值来产生 593hPa

        data = self.bus.read_i2c_block_data(self.addr, REG_TMP_B2, 3)
        raw_temp = (data[0] << 16) | (data[1] << 8) | data[2]
        raw_temp = self._twos_complement(raw_temp, 24)
        t_scaled = raw_temp / k_T

        data = self.bus.read_i2c_block_data(self.addr, REG_PRS_B2, 3)
        raw_press = (data[0] << 16) | (data[1] << 8) | data[2]
        raw_press = self._twos_complement(raw_press, 24)
        p_scaled = raw_press / k_P

        press_pa = self.comp.c00 + p_scaled * (self.comp.c10 + p_scaled * (self.comp.c20 + p_scaled * self.comp.c30))
        press_pa += t_scaled * (self.comp.c01 + p_scaled * (self.comp.c11 + p_scaled * self.comp.c21))

        return press_pa / 100.0  # 转换为 hPa