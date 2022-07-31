# MIT License
# 
# Copyright (c) 2022 INOUE MINORU
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITYz, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import string
import serial
from CalcLidar import calclidar
import time


class YDLidar_GS2:

    port: string
    ser: serial
    cl: calclidar

    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port,
                                 baudrate=921600,
                                 timeout=5.0,
                                 bytesize=8,
                                 parity='N',
                                 stopbits=1)
        self.stoplidar()
        self.getCalcData()

    def stoplidar(self):
        self.ser.write([0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x64, 0x00, 0x00, 0x64])

        packetHeader = b'\x00\x00\x00\x00'
        while packetHeader != b'\xa5\xa5\xa5\xa5':
            packetHeader = self.ser.read(size=4)

        address = self.ser.read(1)
        commandType = self.ser.read(1)
        dataLength = self.ser.read(2)
        checkCode = self.ser.read(1)

        addressNumber = int.from_bytes(address, "little")
        commandTypeNumber = int.from_bytes(commandType, "little")
        dataLengthNumber = int.from_bytes(dataLength, "little")
        checkCodeNumber = int.from_bytes(checkCode, "little")

        return addressNumber, commandTypeNumber, dataLengthNumber, checkCodeNumber

    def startlidar(self):
        self.ser.write([0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x63, 0x00, 0x00, 0x63])
        # 最初の一回のみデータ無しのパケットが返される
        datas = self.ser.read(9)
        return datas

    def getCalcData(self):
        self.ser.write([0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x61, 0x00, 0x00, 0x61])
        datas = self.ser.read(18)
        packetHeaderNumber = hex(int.from_bytes(datas[0:4], "little"))
        addressNumber = hex(datas[4])
        commandTypeNumber = hex(datas[5])
        dataLengthNumber = int(datas[7] << 8 | datas[6])
        K0 = float(datas[9] << 8 | datas[8])
        B0 = float(datas[11] << 8 | datas[10])
        K1 = float(datas[13] << 8 | datas[12])
        B1 = float(datas[15] << 8 | datas[14])

        self.cl = calclidar(K0, B0, K1, B1, datas[16])

        checkCode = hex(datas[17])

        return checkCode, packetHeaderNumber, addressNumber, commandTypeNumber, dataLengthNumber

def getData(self):
        while(1):
            datas = self.ser.read(331)

            if(self.cl.dataCheck(datas) != True):
                print("Data corrupted. Rerun lidar...")
                self.stoplidar()
                self.startlidar()
                continue

            break

        return self.cl.receiveDataCalc(datas)
