# klippy/extras/ch224q_pd.py
# CH224Q uSB Power Delivery Controller support for Klipper (I2C)
#
# Copyright (C) 2025 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

# Registers from CH224Q V2.0 datasheet
REG_STATUS = 0x09              # 8-bit status (read-only)
REG_VOLT_REQ = 0x0A            # 8-bit voltage code
REG_CURR_MAX = 0x50            # Current max (read-only, 50mA)
REG_AVS_HIGH = 0x51            # AVS high 8 bits
REG_AVS_LOW = 0x52             # AVS low 8 bits
REG_PPS_VOLT = 0x53            # PPS voltage (100mV units, 8-bit)
REG_SRCCAP_START = 0x60        # PD/EPR SrcCap (48 bytes, read-only)

# Voltage code mapping (mV for internal use)
VOLTAGE_CODES = {
    5000: 0x00,   # 5V
    9000: 0x01,   # 9V
    12000: 0x02,  # 12V
    15000: 0x03,  # 15V
    20000: 0x04,  # 20V
    28000: 0x05,  # 28V (EPR)
}

class CH224QPD:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]  # e.g., toolhead
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=0x22, default_speed=100000)
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command('PD_SET', self.cmd_PD_SET,
            desc="Set PD voltage (e.g., PD_SET MCU=toolhead "
            "VOLTAGE=9 MODE=FIXED)")
        gcode.register_command('PD_GET', self.cmd_PD_GET,
            desc="Get PD status (e.g., PD_GET MCU=toolhead)")
        gcode.register_command('PD_CAPS', self.cmd_PD_CAPS,
            desc="Dump PD SrcCap (e.g., PD_CAPS MCU=toolhead)")

    def _write_reg8(self, reg, value):
        self.i2c.i2c_write([reg, value & 0xFF])

    def _write_reg16(self, reg_low, value):
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        self._write_reg8(reg_low, high)
        self._write_reg8(reg_low + 1, low)

    def _read_reg8(self, reg):
        resp = self.i2c.i2c_read([reg], 1)
        return resp['response'][0]

    def _read_reg16(self, reg_low):
        resp = self.i2c.i2c_read([reg_low], 2)
        data = resp['response']
        return (data[1] << 8) | data[0]  # Little-endian

    def _read_src_cap(self):
        resp = self.i2c.i2c_read([REG_SRCCAP_START], 48)
        return resp['response']

    def cmd_PD_SET(self, gcmd):
        mcu = gcmd.get('MCU', self.name)
        if mcu != self.name:
            return
        voltage = gcmd.get_float('VOLTAGE', 5.0) * 1000  # Volts to mV
        mode = gcmd.get('MODE', 'FIXED').upper()
        if mode in ('AVS', 'PPS') and not (3300 <= voltage <= 21000):
            raise gcmd.error("VOLTAGE 3.3-21.0 V for AVS/PPS")
        elif mode == 'FIXED' and voltage not in VOLTAGE_CODES:
            raise gcmd.error("VOLTAGE must be one of %s" % [
                v/1000 for v in VOLTAGE_CODES.keys()])
        status = self._read_reg8(REG_STATUS)
        if mode == 'AVS' and not (status & 0x40):
            raise gcmd.error("AVS not supported")
        if mode == 'AVS':
            self._write_reg16(REG_AVS_HIGH, voltage // 25)  # 25mV units
            self._write_reg8(REG_VOLT_REQ, 0x07)
        elif mode == 'PPS':
            self._write_reg8(REG_PPS_VOLT, voltage // 100)  # 100mV units
            self._write_reg8(REG_VOLT_REQ, 0x06)
        else:
            self._write_reg8(REG_VOLT_REQ, VOLTAGE_CODES[voltage])
        gcmd.respond_info("PD requested: %.1fV (%s)" % (voltage / 1000.0, mode))

    def cmd_PD_GET(self, gcmd):
        mcu = gcmd.get('MCU', self.name)
        if mcu != self.name:
            return
        status = self._read_reg8(REG_STATUS)
        active_protocols = []
        if status & 0x01: active_protocols.append("BC")
        if status & 0x02: active_protocols.append("QC2")
        if status & 0x04: active_protocols.append("QC3")
        if status & 0x08: active_protocols.append("PD")
        if status & 0x10: active_protocols.append("EPR")
        epr_exist = "yes" if status & 0x20 else "no"
        avs_exist = "yes" if status & 0x40 else "no"
        power_good = "yes" if status & 0x80 else "no"
        raw_curr = self._read_reg16(REG_CURR_MAX)
        max_curr_ma = raw_curr * 50 if status & 0x08 else 0
        # Read voltage (0x0A first, then 0x53 or 0x51-0x52 if needed)
        voltage_mv = 0
        raw_volt = self._read_reg8(REG_VOLT_REQ)
        if raw_volt == 0x06 and status & 0x08:  # PPS
            raw_pps = self._read_reg8(REG_PPS_VOLT)
            if 3300 <= raw_pps * 100 <= 21000:
                voltage_mv = raw_pps * 100
        elif raw_volt == 0x07 and status & 0x40:  # AVS
            raw_avs = self._read_reg16(REG_AVS_HIGH)
            if 3300 <= raw_avs * 25 <= 21000:
                voltage_mv = raw_avs * 25
        else:
            for v, code in VOLTAGE_CODES.items():
                if raw_volt == code:
                    voltage_mv = v
                    break
        voltage = "%.1f" % (voltage_mv / 1000.0) if voltage_mv else "unknown"
        gcmd.respond_raw(
            "voltage:%s current:%.1f protocols:%s epr_support:%s "
            "avs_support:%s power_good:%s" % (
                voltage,
                max_curr_ma / 1000.0,
                ",".join(active_protocols) or "none",
                epr_exist,
                avs_exist,
                power_good
            )
        )

    def cmd_PD_CAPS(self, gcmd):
        mcu = gcmd.get('MCU', self.name)
        if mcu != self.name:
            return
        data = self._read_src_cap()
        if data and len(data) >= 2 and (self._read_reg8(REG_STATUS) & 0x08):
            # Parse 2-byte header (little-endian)
            header = (data[1] << 8) | data[0]
            msg_type = header & 0x1F
            pd_role = (header >> 5) & 0x01
            spec_rev = (header >> 6) & 0x03
            pr_role = (header >> 8) & 0x01
            msg_id = (header >> 9) & 0x07
            num_do = (header >> 12) & 0x07
            ext = (header >> 15) & 0x01
            report = (
                "PD/EPR SrcCap:\n"
                "Header:\n"
                "  MsgType: 0x%02X (Source_Capabilities)\n"
                "  PDRole: %s\n"
                "  SpecRev: %s\n"
                "  PRRole: %s\n"
                "  MsgID: %d\n"
                "  NumDO: %d\n"
                "  Ext: %d" % (
                    msg_type,
                    "Sink" if pd_role else "Source",
                    ["1.0", "2.0", "3.0"][spec_rev] if spec_rev <= 2
                        else "Unknown",
                    "Sink" if pr_role else "Source",
                    msg_id,
                    num_do,
                    ext
                )
            )
            offset = 2 if ext == 0 else 4
            if ext == 1 and len(data) >= 4:
                ext_header = (data[3] << 8) | data[2]
                data_size = ext_header & 0x1FF
                num_do = data_size // 4
                report += (
                    "\nExtended Header:\n"
                    "  DataSize: %d bytes (%d PDOs)\n"
                    "  Chunked: %d\n"
                    "  ChunkNumber: %d\n"
                    "  RequestChunk: %d\n"
                    "  Rev: %d" % (
                        data_size,
                        num_do,
                        (ext_header >> 15) & 0x01,
                        (ext_header >> 11) & 0x0F,
                        (ext_header >> 10) & 0x01,
                        (ext_header >> 9) & 0x01
                    )
                )
            # Parse PDOs
            pdo_list = []
            for i in range(min(num_do, 7)):
                idx = offset + i * 4
                if idx + 3 >= len(data):
                    report += "\nWarning: Incomplete PDO at offset %d" % idx
                    break
                pdo = (data[idx+3] << 24) | (data[idx+2] << 16
                      ) | (data[idx+1] << 8) | data[idx]
                fixed_supply = (pdo >> 30) & 0x03
                if fixed_supply != 0b11:  # Fixed PDO
                    max_current = (pdo & 0x3FF) * 10
                    voltage = ((pdo >> 10) & 0x3FF) * 50
                    peak_current = (pdo >> 20) & 0x03
                    epr_mode_cap = (pdo >> 23) & 0x01
                    unchunked = (pdo >> 24) & 0x01
                    dual_role_data = (pdo >> 25) & 0x01
                    usb_com_cap = (pdo >> 26) & 0x01
                    unconstrained_pwr = (pdo >> 27) & 0x01
                    usb_suspend = (pdo >> 28) & 0x01
                    dual_role_pwr = (pdo >> 29) & 0x01
                    pdo_str = (
                        "PDO%d: %.1fV, %.1fA, PeakCurrent=%d, EPRModeCap=%d, "
                        "Unchunked=%d, DualRoleData=%d, USBComCap=%d, "
                        "UnconstrainedPWR=%d, USBSuspend=%d, "
                        "DualRolePWR=%d, FixedSupply=%d" % (
                            i + 1,
                            voltage / 1000.0,
                            max_current / 1000.0,
                            peak_current,
                            epr_mode_cap,
                            unchunked,
                            dual_role_data,
                            usb_com_cap,
                            unconstrained_pwr,
                            usb_suspend,
                            dual_role_pwr,
                            fixed_supply
                        )
                    )
                else:  # PPS or AVS PDO
                    apdo = (pdo >> 30) & 0x03
                    pps = (pdo >> 28) & 0x03
                    if pps == 0b01 and apdo == 0b11:  # PPS
                        max_current = (pdo & 0x7F) * 50
                        min_voltage = ((pdo >> 8) & 0xFF) * 100
                        max_voltage = ((pdo >> 17) & 0xFF) * 100
                        pps_pwr_limited = (pdo >> 27) & 0x01
                        pdo_str = (
                            "PDO%d: %.1f-%.1fV (PPS), %.1fA, "
                            "PPSPWRLimited=%d, PPS=%d, APDO=%d" % (
                                i + 1,
                                min_voltage / 1000.0,
                                max_voltage / 1000.0,
                                max_current / 1000.0,
                                pps_pwr_limited,
                                pps,
                                apdo
                            )
                        )
                    elif (pdo >> 27) & 0x03 == 0b01 and apdo == 0b11:  # AVS
                        pdp = pdo & 0xFF
                        min_voltage = ((pdo >> 8) & 0xFF) * 100
                        max_voltage = ((pdo >> 17) & 0x1FF) * 100
                        peak_current = (pdo >> 26) & 0x03
                        pdo_str = ("PDO%d: %.1f-%.1fV (AVS), PDP=%dW, "
                            "PeakCurrent=%d, EPRAVS=%d, APDO=%d" % (
                                i + 1,
                                min_voltage / 1000.0,
                                max_voltage / 1000.0,
                                pdp,
                                peak_current,
                                (pdo >> 27) & 0x03,
                                apdo
                            )
                        )
                    else:
                        pdo_str = "PDO%d: Invalid APDO (PPS=%d, APDO=%d)"%(
                            i + 1, pps, apdo)
                pdo_list.append(pdo_str)
            report += "\nPDOs: %d\n%s" % (len(pdo_list), "\n".join(pdo_list))
        else:
            report = "PD/EPR SrcCap:\n"
            report +="PDOs: 0 (PD mode required or data invalid)"
        gcmd.respond_info(report)

def load_config_prefix(config):
    return CH224QPD(config)
