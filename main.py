import serial
import matplotlib.pyplot as plt
from CalcLidar import calclidar
import time
import cmath


# note: グラフ化して確認したい際にコメントアウト解除する。
# fig = plt.figure(figsize=(8,8))
# ax = fig.add_subplot(111, projection='polar')
# ax.set_theta_direction(-1)
# ax.set_theta_zero_location('N')
# ax.set_title('lidar (exit: Key E)',fontsize=18)
# plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)


ser = serial.Serial(port='/dev/tty.SLAB_USBtoUART',
                    baudrate=921600,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

def stoplidar(ser):
    ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x64,0x00,0x00,0x64])

    packetHeader = b'\x00\x00\x00\x00'
    while packetHeader !=b'\xa5\xa5\xa5\xa5':
        packetHeader = ser.read(size=4)

    address = ser.read(1)
    commandType = ser.read(1)
    dataLength = ser.read(2)
    checkCode = ser.read(1)

    addressNumber = int.from_bytes(address,"little")
    commandTypeNumber = int.from_bytes(commandType,"little")
    dataLengthNumber = int.from_bytes(dataLength,"little")
    checkCodeNumber = int.from_bytes(checkCode,"little")

def startlidar(ser):
    ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x63,0x00,0x00,0x63])
    # 最初の一回のみデータ無しのパケットが返される
    datas = ser.read(9)
    print(f"{datas},")


def getCalcData(ser):
    ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x61,0x00,0x00,0x61])
    datas = ser.read(18)
    packetHeaderNumber = hex(int.from_bytes(datas[0:4],"little"))
    addressNumber = hex(datas[4])
    commandTypeNumber = hex(datas[5])
    dataLengthNumber = int(datas[7]<<8 | datas[6])
    K0 = float(datas[9]<<8 | datas[8])
    B0 = float(datas[11]<<8 | datas[10])
    K1 = float(datas[13]<<8 | datas[12])
    B1 = float(datas[15]<<8 | datas[14])

    cl = calclidar(K0,B0,K1,B1,datas[16])

    checkCode = hex(datas[17])

    print(f"d_compensateK0:{cl.d_compensateK0},\
        d_compensateB0:{cl.d_compensateB0},\
        d_compensateK1:{cl.d_compensateK1},\
        d_compensateB1:{cl.d_compensateB1},\
        bias:{cl.bias}")
    
    return cl


stoplidar(ser)
cl = getCalcData(ser)
startlidar(ser)

while True:
    datas = ser.read(331)
    
    if(cl.dataCheck(datas)!=True):
        ser.close()
        time.sleep(0.1)
        ser = serial.Serial(port='/dev/tty.SLAB_USBtoUART',
                    baudrate=921600,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)
        time.sleep(0.1)

        stoplidar(ser)
        time.sleep(0.1)
        cl = getCalcData(ser)
        startlidar(ser)
        continue

    rt = cl.receiveDataCalc(datas)

    ltheta = rt.thetas[:80]
    rtheta = rt.thetas[80:]
    
    # note: グラフ化した際にコメントアウトを解除する。
    # if('line' in locals()):
    #     line.remove()
    # if('line2' in locals()):
    #     line2.remove()
    # line = ax.scatter(ltheta, rt.distance[:80], c="red", s=5)
    # line2 = ax.scatter(rtheta, rt.distance[80:], c="blue", s=5)
    # ax.set_theta_offset(cmath.pi / 2)
    # ax.set_theta_offset(cmath.pi / 2)
    # plt.pause(0.00001)
    


ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x64,0x00,0x00,0x64])
ser.close()



