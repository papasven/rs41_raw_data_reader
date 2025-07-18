#!/usr/bin/python3
"""
    Read raw hex frames/data from RS41

    Raw hex files from RS41-Tracker rs41mod radiosonde_auto_rx goes.
    Subframe files from radiosonde_auto_rx is ok

    Some things come from here.
    https://github.com/rs1729/RS
    https://github.com/einergehtnochrein/ra-firmware/blob/master/src/rs41/rs41private.h many variables
    https://github.com/bazjo/RS41_Decoding
    https://github.com/projecthorus/radiosonde_auto_rx
    https://github.com/dl9rdz/rdz_ttgo_sonde

    Meterological block is not yet fully functional.
    A lot of things are definitely not right, but a lot of things are ok. ;)
"""

import json
import datetime
import time
import struct
import math
import glob
import os

header = '8635f44093df1a60'
auxtyp = {
    0x01:'Ozonesonde',
    0x05:'OIF411',
    0x08:'CFH',
    0x10:'FPH',
    0x19:'COBALD',
    0x28:'SLW',
    0x38:'POPS',
    0x39:'OPC',
    0x3A:'WVS',
    0x3C:'PCFH',
    0x3D:'FLASH-B',
    0x3E:'TRAPS',
    0x3F:'SKYDEW',
    0x41:'CICANUM',
    0x45:'POPS'
}
frametyp = {
    0x76:'empty',
    0x79:'status',
    0x7A:'meas',
    0x7B:'gpspos',
    0x7C:'gpsinfo',
    0x7D:'gpsraw',
    0x7E:'xdata',
    0x7F:'measshort',
    0x80:'crypt',
    0x82:'gpsposn',
    0x83:'gpssat',
    0x96:'auth'
}
crc16_ccitt_table = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]

def geo(x,y,z):
    x = float(x) #in meters
    y = float(y) #in meters
    z = float(z) #in meters
    a = 6378137.0 #in meters
    b = 6356752.314245 #in meters
    f = (a - b) / a
    f_inv = 1.0 / f
    e_sq = f * (2 - f)                       
    eps = e_sq / (1.0 - e_sq)
    p = math.sqrt(x * x + y * y)
    q = math.atan2((z * a), (p * b))
    sin_q = math.sin(q)
    cos_q = math.cos(q)
    sin_q_3 = sin_q * sin_q * sin_q
    cos_q_3 = cos_q * cos_q * cos_q
    phi = math.atan2((z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3))
    lam = math.atan2(y, x)
    v = a / math.sqrt(1.0 - e_sq * math.sin(phi) * math.sin(phi))
    h   = (p / math.cos(phi)) - v
    lat = math.degrees(phi)
    lon = math.degrees(lam)
    return lat,lon,h

def crc_ccitt_16(data):
    """
    Compute CRC-16-CCITT using precomputed table.
    https://www.askpython.com/python/examples/crc-16-bit-manual-calculation
    """
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) ^ crc16_ccitt_table[(crc >> 8) ^ byte]) & 0xFFFF
    return crc

def flags(f):
    r = '';
    KnownBits = 0x0|0x1|0x2|0x3|0xc|0x8c
    if(f&~KnownBits):r += (f"unknown 0x{f&~KnownBits:02x}|");
    if(f&0x1):r += f"Pressure Sensor 0x{0x1:02x}|"
    if(f&0x2):r += f"0x{0x2:02x}|"
    if(f&0x3):r += f"GPS 0x{0x3:02x}|"
    if(f&0xc):r += f"0x{0xc:02x}|"
    if(f&0x8c):r += f"0x{0x8c:02x}|"
    return r

def bit_reserved00B(b):
    ba = {}
    bf=[b[i//8] & 1 << i%8 != 0 for i in range(len(b) * 8)]
    ba["crcerror"] = 'CRC error user parameters' if bf[4] == True else None
    ba["nfcfield"] = 'NFC field' if bf[9] == True else None
    return ba

def bit_flags(b):
    ba = {}
    bf=[b[i//8] & 1 << i%8 != 0 for i in range(len(b) * 8)]
    ba["mode"] = 'Flight mode' if bf[0] == True else 'Start phase'
    ba["cent"] = 'Descent' if bf[1] == True else 'Ascent'
    ba["check"] = 'self-check' if bf[5] == True else None
    ba["VBATmin"] = 'VBATmin check enabled' if bf[11] == True else 'VBATmin check disabled'
    ba["VBAT"] = 'low' if bf[12] == True else 'ok'
    return ba

def error_log(f):
    r = '';
    KnownBits = 0x000|0x001|0x002|0x003|0x004|0x005|0x006|0x007|0x008|0x009|0x00a|0x00b|0x00c|0x00d|0x00e|0x040|0x200
    errorlist = {0x000:"Low battery capacity",
               0x001:"No parameter setup",
               0x002:"TX init failure",
               0x003:"Not in TX state",
               0x004:"No add-on sensor data",
               0x005:"Various flash errors",
               0x006:"PTU failure",
               0x007:"GPS init failure",
               0x008:"Invalid GPS messages",
               0x009:"Missing GPS messages",
               0x00a:"T self-check failure",
               0x00b:"U self-check failure",
               0x00c:"Low regen temperature",
               0x00d:"P-module not detected",
               0x00e:"T, Tu or U check failed",
               0x040:"No sensor data?",
               0x200:"GPS error?"}
    if(f&~KnownBits):r += (f"unknown error 0x{f&~KnownBits:03x}|");
    for kB in errorlist:
        if f&kB:r += f"{errorlist[kB]} 0x{kB:03x}|"
    return r

def t(gpsweek,timeOfWeek):
    second_before = 315964800 #unix epoch 01.01.1970 to gps epoch 06.01.1980
    seconds_week = 604800
    second_after = gpsweek * seconds_week + timeOfWeek
    return  (second_before+second_after)

def subframe32(s):
    x = bytearray.fromhex(s)
    a={}
    try:
        a['killCountdown'] = struct.unpack('<h',(x[0x320-0x320:0x322-0x320]))[0]
        a['launchAltitude'] = struct.unpack('<h',(x[0x322-0x320:0x324-0x320]))[0]
        a['heightOfFlightStart'] = x[0x324-0x320]|x[0x325-0x320]<<8
        a['lastTxPowerLevel'] = x[0x326-0x320]
        a['numSoftwareResets'] = x[0x327-0x320]
        a['intTemperatureCpu'] = struct.unpack('<b',(x[0x328-0x320:0x329-0x320]))[0]
        a['intTemperatureRadio'] = struct.unpack('<b',(x[0x329-0x320:0x32a-0x320]))[0]
        a['remainingBatteryCapacity'] = x[0x32A-0x320]|x[0x32B-0x320]<<8
        a['numUbxDiscarded'] = x[0x32C-0x320]
        a['numUbxStall'] = x[0x32D-0x320]
    except Exception as e:
        pass
        print(f"error_subframe32:{e}")
    return a

def subframe(d,only_subframe_file = False):
    strf = '\x00'
    a = {}
    try:
        if only_subframe_file:
            f = open(d, "rb")
            b = f.read()
            x = bytearray(b)
        else:
            x = d
        lbb = len(x)
        crcv = x[0x000]|x[0x001]<<8
        crcs = crc_ccitt_16(x[0x002:lbb-16])
        if crcv == crcs: a['subframe_crc']=1 
        else: a['subframe_crc']=0
        a['freq'] = int(round(400+(x[0x003]+(x[0x002]/255))*0.03999938486145,3)*1000000)
        a['startupTxPower'] = x[0x004]
        a['reserved005'] = x[0x005]|x[0x006]<<8
        a['optionFlags'] = flags(x[0x007]|x[0x008]<<8)
        a['reserved009'] = x[0x009]|x[0x00a]<<8
        a['reserved00B'] = x[0x00b]|x[0x00c]<<8
        a['serial'] = str(x[0x00d:0x015], 'ascii', errors='ignore')
        a['firmware'] = x[0x015]|x[0x016]<<8|x[0x017]<<16|x[0x018]<<24
        a['minHeight4Flight'] = x[0x019]|x[0x01a]<<8
        a['lowBatVoltageThreshold'] = x[0x01b]
        a['nfcDetectorThreshold'] = x[0x01c]
        a['reserved01D'] = x[0x01d]
        a['reserved01E'] = x[0x01e]
        a['reserved01F'] = x[0x01f]|x[0x020]<<8
        a['refTemperatureTarget'] = x[0x021]
        a['lowBatCapacityThreshold'] = x[0x022]
        a['reserved023'] = x[0x023]|x[0x024]<<8
        a['reserved025'] = x[0x025]|x[0x026]<<8
        a['flightKillFrames'] = struct.unpack('<h',(x[0x027:0x029]))[0]
        a['reserved029'] = x[0x029]|x[0x02a]<<8
        a['burstKill'] = x[0x02b]
        a['reserved02C'] = x[0x02c]
        a['reserved02D'] = x[0x02d]
        a['freshBatteryCapacity'] = x[0x02e]|x[0x02f]<<8
        a['reserved030'] = x[0x030]|x[0x031]<<8
        a['allowXdata'] = x[0x032]
        a['ubloxHwVersionHigh'] = x[0x033]|x[0x034]<<8
        a['ubloxHwVersionLow'] = x[0x035]|x[0x036]<<8
        a['ubloxSwVersion'] = x[0x037]|x[0x038]<<8
        a['ubloxSwBuild'] = x[0x039]|x[0x03a]<<8
        a['ubloxConfigErrors'] = x[0x03b]
        a['radioVersionCode'] = x[0x03c]
        a['refResistorLow'] = struct.unpack('<f', x[0x03d:0x041])[0]
        a['refResistorHigh'] = struct.unpack('<f', x[0x041:0x045])[0]
        a['refCapLow'] = struct.unpack('<f', x[0x045:0x049])[0]
        a['refCapHigh'] = struct.unpack('<f', x[0x049:0x04d])[0]
        a['taylorT'] = []
        for c in range(3):a['taylorT'].append(struct.unpack('<f', x[0x04d+(4*c):0x051+(4*c)])[0])
        a['calT'] = struct.unpack('<f', x[0x059:0x05D])[0]
        a['polyT'] = []
        for c in range(6):a['polyT'].append(struct.unpack('<f', x[0x05D+(4*c):0x061+(4*c)])[0])
        a['calibU'] = []
        for c in range(2):a['calibU'].append(struct.unpack('<f', x[0x075+(4*c):0x079+(4*c)])[0])
        a['matrixU'] = {}
        for b in range(7):
            a['matrixU'][b] = []
            for c in range(6):
                a['matrixU'][b].append(struct.unpack('<f', x[0x07D+(4*c)+(21*b):0x081+(4*c)+(21*b)])[0])
        a['taylorTU'] = []
        for c in range(3):a['taylorTU'].append(struct.unpack('<f', x[0x125+(4*c):0x129+(4*c)])[0])
        a['calTU']=struct.unpack('<f', x[0x131:0x135])[0]
        a['polyTrh'] = []
        for c in range(3):a['polyTrh'].append(struct.unpack('<f', x[0x135+(4*c):0x139+(4*c)])[0])
        a['reserved14D'] = x[0x14D]
        a['reserved14E'] = x[0x14E]|x[0x14F]<<8|x[0x150]<<16|x[0x151]<<24
        a['f152'] = struct.unpack('<f', x[0x152:0x156])[0]
        a['u156'] = x[0x156]
        a['f157'] = struct.unpack('<f', x[0x157:0x15b])[0]
        a['reserved15B'] = x[0x15b]
        a['reserved15C'] = x[0x15C]|x[0x15d]<<8|x[0x15e]<<16|x[0x15f]<<24
        a['f160'] = []
        for c in range(35):a['f160'].append(struct.unpack('<f', x[0x160+(4*c):0x164+(4*c)])[0])
        a['startIWDG'] = x[0x1EC]
        a['parameterSetupDone'] = x[0x1ED]
        a['enableTestMode'] = x[0x1EE]
        a['enableTX'] = x[0x1EF]
        a['f1F0'] = []
        for c in range(8):a['f1F0'].append(struct.unpack('<f', x[0x1F0+(4*c):0x1F4+(4*c)])[0])
        a['pressureLaunchSite'] = []
        for c in range(2):a['pressureLaunchSite'].append(struct.unpack('<f', x[0x210+(4*c):0x214+(4*c)])[0])
        a['variant'] = str(x[0x218:0x222], 'ascii', errors='ignore').strip(strf)
        a['mainboard'] = str(x[0x222:0x22C], 'ascii', errors='ignore').strip(strf)
        a['mainboardSerial'] = str(x[0x22C:0x235], 'ascii', errors='ignore').strip(strf)
        a['text235'] = str(x[0x235:0x23f], 'ascii', errors='ignore').strip(strf)
        a['reserved23f'] = x[0x23f]|x[0x240]<<8# if pressure Sensor?
        a['reserved241'] = x[0x241]|x[0x242]<<8
        a['pressureSensorSerial'] = str(x[0x243:0x24B], 'ascii', errors='ignore').strip(strf)
        a['reserved24B'] = str(x[0x24B:0x24D], 'ascii', errors='ignore').split(strf)
        a['reserved24D'] = x[0x24D]|x[0x02e]<<8
        a['reserved24F'] = x[0x24F]|x[0x250]<<8
        a['reserved251'] = x[0x251]|x[0x252]<<8
        a['xdataUartBaud'] = x[0x253]
        a['reserved254'] = x[0x254]
        a['cpuTempSensorVoltageAt25deg'] = struct.unpack('<f', x[0x255:0x259])[0]
        a['reserved259'] = x[0x259]
        a['reserved25A'] = x[0x25A]
        a['reserved25B'] = x[0x25B]
        a['reserved25C'] = x[0x25C]
        a['reserved25D'] = x[0x25D]
        a['matrixP'] = []
        for c in range(18):a['matrixP'].append(struct.unpack('<f', x[0x25E+(4*c):0x262+(4*c)])[0])
        a['vectorBp'] = []
        for c in range(3):a['vectorBp'].append(struct.unpack('<f', x[0x2A6+(4*c):0x2AA+(4*c)])[0])
        a['reserved2B2'] = []
        for c in range(8):a['reserved2B2'].append(struct.unpack('<f', x[0x2B2+(4*c):0x2B6+(4*c)])[0])
        a['matrixBt'] = []
        for c in range(12):a['matrixBt'].append(struct.unpack('<f', x[0x2BA+(4*c):0x2BE+(4*c)])[0])
        a['reserved2EA'] = []
        for c in range(16):
            a['reserved2EA'].append(x[0x2EA+c])
        a['halfword2FA'] = []
        for c in range(9):a['halfword2FA'].append(x[0x2FA+(2*c)]|x[0x2FB+(2*c)]<<8)
        a['reserved30C'] = struct.unpack('<f', x[0x30C:0x310])[0]
        a['reserved310'] = struct.unpack('<f', x[0x310:0x314])[0]
        a['reserved314'] = x[0x314]
        a['reserved315'] = x[0x315]
        a['burstKillFrames'] = struct.unpack('<h',(x[0x316:0x318]))[0]
        a['reserved318'] = []
        for c in range(8):
            a['reserved318'].append(x[0x318+c])
        a['killCountdown'] = struct.unpack('<h',(x[0x320:0x322]))[0]
        a['launchAltitude'] = struct.unpack('<h',(x[0x322:0x324]))[0]
        a['heightOfFlightStart'] = x[0x324]|x[0x325]<<8
        a['lastTxPowerLevel'] = x[0x326]
        a['numSoftwareResets'] = x[0x327]
        a['intTemperatureCpu'] = struct.unpack('<b',(x[0x328:0x329]))[0]
        a['intTemperatureRadio'] = struct.unpack('<b',(x[0x329:0x32a]))[0]
        a['remainingBatteryCapacity'] = x[0x32A]|x[0x32B]<<8
        a['numUbxDiscarded'] = x[0x32C]
        a['numUbxStall'] = x[0x32D]
        if only_subframe_file:
            #search for softwarerest in subframe #testing
            #if 'numSoftwareResets' in a and a['numSoftwareResets']>0:
                #print(f"SoftwareResets_{a['serial']}: {a['numSoftwareResets']}")
            return f"\033[0;96msubframe:{json.dumps(a)}\033[0m\n"
    except Exception as e:
        print(f"error_subframe:{e}")
    return a

def auth(i,block_crc):
    """test attempt"""
    auth = {}
    try:
        lbb=len(i)
        if lbb == 12: #sometimes dontnow
            auth['auth_crc'] = block_crc
            auth['reserved000'] = i[0x000]
            auth['reserved001'] = i[0x001]
            val = 0x002
            rangeval=lbb-val-2 #-2 crc ignore
            strv =''
            for z in range(rangeval):
               strv += f"{i[val+z]:02X}"
            auth['auth_value'] = strv
    except Exception as e:
        print(f"error_auth:{e} dict:{auth} len:{lbb}")
    return auth

def status(x,block_crc):
    s = {}
    try:
        lbb = len(x)
        if lbb == 42: #sometimes 23
            s['status_crc'] = block_crc
            if x[0x000]|x[0x001]<<8|x[0x002]<<16|x[0x003]<<24 == 0: return s #empty
            s['frame'] = x[0x000]|x[0x001]<<8
            s['id'] = str(x[0x002:0x00a], 'ascii', errors='ignore')
            s['batt'] = x[0x00a]/10
            s['reserved00B'] = bit_reserved00B(x[0x00b:0x00d])
            s['flags'] = bit_flags(x[0x00d:0x00f])
            s['typed'] = x[0x00f]
            if x[0x00f] == 0:s['type'] = 'RS41'
            elif x[0x00f] == (1 or 2):s['type'] = 'RS41-SGM'
            elif x[0x00f] == (3 or 4):s['type'] = 'RS41-SGM encrypted' 
            elif x[0x00f] == 6:s['type'] = 'unknown' 
            s['temperatureRef'] = x[0x010]
            s['errorLog'] = x[0x011]|x[0x012]<<8
            s['humidityHeatingPwm'] = x[0x013]|x[0x014]<<8
            s['txPower'] = x[0x015]
            s['maxCalibIndex'] = x[0x016]
            s['thisCalibIndex'] = x[0x017]
            calibFragment = x[0x018:0x018+16]
            hcf=''
            for h in calibFragment:
                hcf += f"{h:02X}"
            s['calibFragment'] = hcf
            sff = f"0x{s['thisCalibIndex']:02X}:"
            s['rs41_subfrm'] = sff + hcf
    except Exception as e:
        print(f"error_status:{e} dict:{s} len:{lbb}")
    return s

def meas(m,block_crc):
    s = {}
    try:
        lbb = len(m)
        if lbb == 44: #sometimes 23
            s['meas_crc'] = block_crc
            if m[0x000]|m[0x001]<<8|m[0x002]<<16|m[0x003]<<24 == 0: return s #empty
            s['main'] = {}
            s['ref1'] = {}
            s['ref2'] = {}
            for x in range(4):
                s['main'][x] = m[0x000+(9*x)]|m[0x001+(9*x)]<<8|m[0x002+(9*x)]<<16
                s['ref1'][x] = m[0x003+(9*x)]|m[0x004+(9*x)]<<8|m[0x005+(9*x)]<<16
                s['ref2'][x] = m[0x006+(9*x)]|m[0x007+(9*x)]<<8|m[0x008+(9*x)]<<16
            s['reserved024'] = m[0x024]|m[0x025]<<8
            s['TemperaturePressureSensor']=(m[0x026]|m[0x027]<<8)/100
            s['reserved028'] = m[0x028]|m[0x029]<<8
    except Exception as e:
        print(f"error_meas:{e} dict:{s} len:{lbb}")
    return s

def measshort(m,block_crc):
    s = {}
    try:
        lbb=len(m)
        if lbb == 29: #sometimes 23
            s['measshort_crc'] = block_crc
            if m[0x000]|m[0x001]<<8|m[0x002]<<16|m[0x003]<<24 == 0: return s
            s['main'] = {}
            s['ref1'] = {}
            s['ref2'] = {}
            for x in range(3):
                s['main'][x] = m[0x000+(9*x)]|m[0x001+(9*x)]<<8|m[0x002+(9*x)]<<16
                s['ref1'][x] = m[0x003+(9*x)]|m[0x004+(9*x)]<<8|m[0x005+(9*x)]<<16
                s['ref2'][x] = m[0x006+(9*x)]|m[0x007+(9*x)]<<8|m[0x008+(9*x)]<<16
    except Exception as e:
        print(f"error_measshort:{e} dict:{s} len:{lbb}")
    return s

def gpspos(bb,block_crc):
    pos = {}
    try:
        lbb = len(bb)
        if lbb == 23 or lbb == 40: #23 new firmware 20701 len(38)40# datetime inside
            pos['gpspos_crc'] = block_crc
            if bb[0x000]|bb[0x001]<<8|bb[0x002]<<16|bb[0x003]<<24 == 0: return pos #empty
            x = (bb[0x000]|bb[0x001]<<8|bb[0x002]<<16|bb[0x003]<<24)/100
            y = (bb[0x004]|bb[0x005]<<8|bb[0x006]<<16|bb[0x007]<<24)/100
            z = (bb[0x008]|bb[0x009]<<8|bb[0x00a]<<16|bb[0x00b]<<24)/100
            xx = (struct.unpack('<h',(bb[0x00c:0x00e]))[0])/100
            yy = (struct.unpack('<h',(bb[0x00e:0x010]))[0])/100
            zz = (struct.unpack('<h',(bb[0x010:0x012]))[0])/100
            pos['xx'] = xx
            pos['yy'] = yy
            pos['zz'] = zz
            pos['lat'], pos['lon'], pos['alt']=geo(x,y,z)
            phi = pos['lat']*math.pi/180.0
            lam = pos['lon']*math.pi/180.0
            pos['vN']  = -xx*math.sin(phi)*math.cos(lam) - yy*math.sin(phi)*math.sin(lam) + zz*math.cos(phi)
            pos['vE'] = -xx*math.sin(lam) + yy*math.cos(lam)
            pos['vU']  =  xx*math.cos(phi)*math.cos(lam) + yy*math.cos(phi)*math.sin(lam) + zz*math.sin(phi)
            pos['vel_h']  = math.sqrt(pos['vN']*pos['vN']+pos['vE']*pos['vE']);
            pos['heading']  = math.atan2(pos['vE'], pos['vN']) * 180 / math.pi;
            if (pos['heading'] < 0): pos['heading'] += 360
            pos['vel_v']  = pos['vU'];
            if lbb == 40:
                pos['year'] = bb[0x012]|bb[0x013]<<8
                pos['month'] = bb[0x014]
                pos['day'] = bb[0x015]
                pos['hour'] = bb[0x016]
                pos['minute'] = bb[0x017]
                pos['second'] = bb[0x018]
                dt = datetime.datetime(pos['year'], pos['month'], pos['day'], pos['hour'], pos['minute'], pos['second'])
                pos['datetime'] = dt.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            if lbb == 23:
                pos['sats'] = bb[0x012]
                pos['acc'] = bb[0x013]
                pos['dop'] = bb[0x014]
            pos['x'] = x
            pos['y'] = y
            pos['z'] = z
    except Exception as e:
        print(f"error_gpspos:{e} dict:{pos} len:{lbb}")
    return pos

def gpsraw(r,block_crc):
    raw = {}
    try:
        lbb = len(r)
        if lbb == 91: #sometimes 32 42
            raw['gpsraw_crc'] = block_crc
            if r[0x000]|r[0x001]<<8|r[0x002]<<16|r[0x003]<<24 == 0: return raw #empty
            raw['minPrMes'] = r[0x000]|r[0x001]<<8|r[0x002]<<16|r[0x003]<<24
            raw['mon_jam'] = r[0x004]
            raw['deltaPrMes'] = {}
            raw['doMes'] = {}
            for x in range(12):
                raw['deltaPrMes'][x]=r[0x005+(7*x)]|r[0x006+(7*x)]<<8|r[0x007+(7*x)]<<16|r[0x008+(7*x)]<<24
                raw['doMes'][x]=r[0x009+(7*x)]|r[0x00a+(7*x)]<<8|r[0x00b+(7*x)]<<16
    except Exception as e:
        pass
        print(f"error_gpsraw:{e} dict:{raw} len:{lbb}")
    return raw

def gpsinfo(i,block_crc):
    info = {}
    try:
        lbb=len(i)
        if lbb == 32: #sometimes dontnow
            info['gpsinfo_crc'] = block_crc
            if i[0x000]|i[0x001]<<8|i[0x002]<<16|i[0x003]<<24 == 0: return info #empty
            info['gpsWeek'] = i[0x000]|i[0x001]<<8
            info['timeOfWeek'] = i[0x002]|i[0x003]<<8|i[0x004]<<16|i[0x005]<<24
            d = datetime.datetime.fromtimestamp(t(info['gpsWeek'],info['timeOfWeek']/1000))
            info['datetime'] = d.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]+ "Z"
            info['prn'] = {}
            info['cno_mesQI'] = {}
            for x in range(12):
                info['prn'][x] = i[0x006+(2*x)]
                info['cno_mesQI'][x] = i[0x007+(2*x)]
    except Exception as e:
        print(f"error_gpsinfo:{e} dict:{info} len:{lbb}")
    return info

def readraw(raw_file,crc_false_show = True,subframe_show = True,subframe32_show = True,json_show = True,frame_fragments = True,debug = True):
    strce = '\033[0m'
    cstr = {}
    serial = ''
    onesub = 0
    onelog = 0
    try:
        f = open(raw_file, "r")

        for l_raw in f:
            l = l_raw.lower().replace(' ','')#RS41-Tracker#some more bytes? 

            if(header.lower() in l):
                frame = {}
                #ecc = l[0x008*2:0x038*2]
                #types = l[0x038*2:0x039*2]
                crc_frame = 1 if '[ok]' in l else 0 #if [ok] frame crc ok
                fpos = l.find("[")#rs41mod
                if fpos == -1:fpos=len(l)
                if fpos % 2:#rarely
                    fpos = fpos - 1
                r = bytearray.fromhex(l[:fpos])
                be = 0x039
                auxstr = ''
                auxc = 0

                while be+1 < len(r):
                    now = time.time()
                    blocktyp = r[be]#first status or new 150
                    blocklen = r[be+1]
                    if blocktyp == 0 and blocklen == 0:break#something wrong
                    if be+blocklen+3 >= len(r):break#something wrong
                    crc = r[be+blocklen+2]|r[be+blocklen+3]<<8
                    crcstr = crc_ccitt_16(r[be+2:be+blocklen+2]) if crc_frame == 0 else crc
                    block_crc = 1 if crc == crcstr else 0
                    if crc == crcstr or crc_false_show:

                        frametyppe_match = frametyp[blocktyp] if blocktyp in frametyp else 0
                        if debug:print(f"blocktyp: {blocktyp} frametyppe_match:{frametyppe_match} blocklen:{blocklen}")

                        match frametyppe_match:
                            case 'status':
                                if crc == crcstr:frame.update(status(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;92m'
                                if frame_fragments:print(f"{strc}status:{status(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                                if 'id' in frame and serial != frame['id'] and crc == crcstr:
                                    serial = frame['id']
                                    cstr = {}
                                    onesub = 0
                                    onelog = 0
                            case 'meas':
                                if crc == crcstr:frame.update(meas(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;93m'
                                if frame_fragments:print(f"{strc}meas:{meas(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case 'measshort':
                                if crc == crcstr:frame.update(measshort(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;93m'
                                if frame_fragments:print(f"{strc}measshort:{measshort(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case 'gpspos'|'gpsposn':
                                if crc == crcstr:frame.update(gpspos(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;94m'
                                if frame_fragments:print(f"{strc}gpspos:{gpspos(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case 'gpsraw':
                                if crc == crcstr:frame.update(gpsraw(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;94m'
                                if frame_fragments:print(f"{strc}gpsraw:{gpsraw(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case 'gpsinfo':
                                if crc == crcstr:frame.update(gpsinfo(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;94m'
                                if frame_fragments:print(f"{strc}gpsinfo:{gpsinfo(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case  'auth': 
                                if crc == crcstr:frame.update(auth(r[be+2:be+blocklen+2+2],block_crc))
                                strc = '\033[0;91m' if crc!=crcstr else '\033[0;96m'
                                if frame_fragments:print(f"{strc}auth:{auth(r[be+2:be+blocklen+2+2],block_crc)}{strce}")
                            case 'xdata':
                                if auxc > 0:auxstr += '#'
                                hca = ''
                                for h in r[be+3:be+blocklen+2]:
                                    hca += chr(h)
                                auxstr += hca
                                auxc += 1
                                if crc == crcstr:frame['aux'] = auxstr
                            case  'empty': 
                                pass
                            case  'crypt': 
                                pass
                                if debug:print(f"crypt:{blocktyp}")
                            case _  : 
                                pass
                                if debug:print(f"unknown:{blocktyp}")

                    be += blocklen + 2 + 2
                    if be >= len(r):break#something wrong

                if auxc > 0: 
                    strc = '\033[0;91m' if crc!=crcstr else '\033[0;92m'
                    if frame_fragments:print(f"{strc}aux:{auxstr}{strce}")

                if 'id' in frame and frame["status_crc"] == 1:
                    cn = frame['thisCalibIndex']#number calib
                    strc = '\033[0;100m'
                    if cn == 50 and subframe32_show:print(f"{strc}32:{subframe32(frame['calibFragment'])}{strce}")
                    if cn not in cstr or cn == 50:cstr[cn] = frame['calibFragment']

                    if(len(cstr) == 51 and cn == 50):#full calib round
                        hvs=''
                        for x in range(51):
                            hvs += cstr[x]
                        hvs = (bytes.fromhex(hvs))
                        frame['subframe'] = subframe(hvs)
                        strc = '\033[0;96m'
                        if subframe_show: print(f"{strc}subframe:{frame['subframe']}{strce}")
                #advanced search

                #search for softwarerest in subframe
                #if'subframe' in frame and frame['subframe']['numSoftwareResets'] > 0 and onesub == 0:
                    #print(f"SoftwareResets_{frame['id']}: {frame['subframe']['numSoftwareResets']}")
                    #onesub = 1
                #search for softwarerest in status frame
                #if'errorLog' in frame and frame['errorLog'] > 0 and onelog == 0:
                    #print(f"errorlog_{frame['id']}: {error_log(frame['errorLog'])}")#testing
                    #onelog=1

                if len(frame) > 0 and json_show: print(f"{json.dumps(frame)}")
    except Exception as e:
        print(f"error__readraw: {e}")

try:
    """e.g. read all *.raw files in the log folder from radiosonde_auto_rx"""
    #rawfileslist = [p for p in glob.glob('/home/user/radiosonde_auto_rx/auto_rx/log/*.raw', recursive = True) if os.path.isfile(p)]
    #for rawfile in rawfileslist:
        #readraw(raw_file = rawfile,crc_false_show = True,subframe_show = False,subframe32_show = False,json_show = False,frame_fragments = False,debug = False)

    readraw(raw_file = 'log/V5230820_raw_frames_reset.log',crc_false_show = False,subframe_show = True,subframe32_show = False,json_show = False,frame_fragments = True,debug = False)
    readraw(raw_file = 'log/V1950381_raw_frames.log',crc_false_show = False,subframe_show = True,subframe32_show = False,json_show = False,frame_fragments = True,debug = False)

    """read only the subframe in log folder from radiosonde_auto_rx"""
    print(subframe('log/20241201-164916_V5250062_RS41-SGP_405100_subframe.bin', only_subframe_file = True))

    """e.g. read all *_subframe.bin files in the log folder from radiosonde_auto_rx"""
    #subframelist = [p for p in glob.glob('/home/user/radiosonde_auto_rx/auto_rx/log/*_subframe.bin', recursive = True) if os.path.isfile(p)]
    #for rawfile in subframelist:
        #print(subframe(rawfile, only_subframe_file = True))
except Exception as e:
    print(f"error:{e}")
