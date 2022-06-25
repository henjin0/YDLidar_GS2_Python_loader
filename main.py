from cmath import pi
import cmath
from tabnanny import check
from turtle import distance
import serial
import binascii
import matplotlib.pyplot as plt
import math
import numpy as np

# init
d_compensateK0 = 0.0
d_compensateB0 = 0.0
d_compensateK1 = 0.0
d_compensateB1 = 0.0
bias = 0.0

angle_p_x =  1.22
angle_p_y = 5.315
angle_p_angle = 22.5

def leftCamThetaCalc(measurement_point):
    measurement_point = 80-measurement_point
    if d_compensateB0 > 1:
        tempTheta = d_compensateK0 * measurement_point - d_compensateB0
    else:
        tempTheta = cmath.atan(d_compensateK0 * measurement_point - d_compensateB0).real * 180 / cmath.pi
    
    return tempTheta

def rightCamThetaCalc(measurement_point):
    measurement_point = 160 - measurement_point 
    if d_compensateB1 > 1:
        tempTheta = d_compensateK1 * measurement_point - d_compensateB1
    else:
        tempTheta = cmath.atan(d_compensateK1 * measurement_point - d_compensateB1).real * 180 / cmath.pi
    
    return tempTheta

def leftCalc(tempDist_,thetas_rad):
    tempDist = (tempDist_ - angle_p_x) / cmath.cos((angle_p_angle +bias - (thetas_rad*180/cmath.pi)) * cmath.pi / 180).real; 
        
    tempX_ = cmath.cos((angle_p_angle + bias) * cmath.pi/ 180).real * tempDist * cmath.cos(thetas_rad).real + \
            cmath.sin((angle_p_angle + bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(thetas_rad).real)
    tempY_ = -cmath.sin((angle_p_angle + bias) * cmath.pi / 180).real * tempDist * cmath.cos(thetas_rad).real +\
            cmath.cos((angle_p_angle+ bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(thetas_rad).real)
    tempX = tempX_ + angle_p_x
    tempY = tempY_ - angle_p_y
    if(tempX != 0.0):
        Dist = cmath.sqrt(tempX * tempX + tempY * tempY).real
        theta = cmath.atan(tempY / tempX).real
    else:
        Dist = 0
        theta = 0
    return Dist,theta

def rightCalc(tempDist_,thetas_rad):
    tempDist = (tempDist_ - angle_p_x) / cmath.cos((angle_p_angle + bias + (thetas_rad*180/cmath.pi)) * cmath.pi / 180).real; 
        
    tempX_ = cmath.cos(-(angle_p_angle + bias) * cmath.pi/ 180).real * tempDist * cmath.cos(thetas_rad).real + \
            cmath.sin(-(angle_p_angle + bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(thetas_rad).real)
    tempY_ = -cmath.sin(-(angle_p_angle + bias) * cmath.pi / 180).real * tempDist * cmath.cos(thetas_rad).real +\
            cmath.cos(-(angle_p_angle+ bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(thetas_rad).real)
    tempX = tempX_ + angle_p_x
    tempY = tempY_ + angle_p_y
    if(tempX != 0.0):
        Dist = cmath.sqrt(tempX * tempX + tempY * tempY).real
        theta = cmath.atan(tempY / tempX).real
    else:
        Dist = 0
        theta = 0
    return Dist,theta


fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_direction(-1)
ax.set_theta_zero_location('N')
ax.set_title('lidar (exit: Key E)',fontsize=18)
plt.connect('key_press_event', lambda event: exit(1) if event.key == 'e' else None)


ser = serial.Serial(port='/dev/tty.SLAB_USBtoUART',
                    baudrate=921600,
                    timeout=5.0,
                    bytesize=8,
                    parity='N',
                    stopbits=1)

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
d_compensateK0 = K0/10000.0
d_compensateB0 = B0/10000.0
d_compensateK1 = K1/10000.0
d_compensateB1 = B1/10000.0
bias = float(datas[16])/10
checkCode = hex(datas[17])

print(f"d_compensateK0:{d_compensateK0},\
    d_compensateB0:{d_compensateB0},\
    d_compensateK1:{d_compensateK1},\
    d_compensateB1:{d_compensateB1},\
    bias:{bias}")

thetas_deg =[] 
thetas_rad =[] 
for i in range(160):
    if i < 80:
        thetas_deg.append(leftCamThetaCalc(i))
        thetas_rad.append(leftCamThetaCalc(i)/180*cmath.pi) 
    else:
        thetas_deg.append(rightCamThetaCalc(i))
        thetas_rad.append(rightCamThetaCalc(i)/180*cmath.pi) 

ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x63,0x00,0x00,0x63])
# 最初の一回のみデータ無しのパケットが返される
datas = ser.read(9)
print(f"{datas},")

while True:
    datas = ser.read(331)

    packetHeaderNumber = hex(int.from_bytes(datas[0:4],"little"))
    addressNumber = hex(datas[4])
    commandTypeNumber = hex(datas[5])
    dataLengthNumber = int(datas[7]<<8 | datas[6])
    ENV = int(datas[9]<<8 | datas[8])

    distance = []
    distance2 = []
    thetas = []
    for i in range(int(len(datas[10:-1])/2)):
        tempDist = (datas[8+2*i+1]<<8 | datas[8+2*i]) & 0x01ff
        
        if i < 80:
            Dist,theta = leftCalc(tempDist,thetas_rad[i])
        else:
            Dist,theta = rightCalc(tempDist,thetas_rad[i])

        if Dist > 300:
            distance2.append(0)
        else:
            distance2.append(Dist)
        distance.append((datas[8+2*i+1]<<8 | datas[8+2*i]) & 0x01ff)
        thetas.append(theta)
        


    
    checkCodeNumber = hex(datas[330])

    # print(f"header:{packetHeaderNumber},\
    #     address:{addressNumber},\
    #     command{commandTypeNumber},\
    #     datalen:{dataLengthNumber},\
    #     datas:{distance},\
    #     checkcode:{checkCodeNumber}")
    
    ltheta = thetas[:80]
    rtheta = thetas[80:]
    # print(f"ltheta:{ltheta}")
    # print(f"rtheta:{rtheta}")
    #ltheta = list(reversed(thetas_rad[0:80]))
    # print(f"ltheta:{list(map(lambda x:x*180/cmath.pi,ltheta))}")
    # print(f"rtheta:{list(map(lambda x:x*180/cmath.pi,rtheta))}")


    # line = ax.scatter(list(map(lambda x:-(x-thetas_rad[79])+25/180*cmath.pi,ltheta)), distance2[:80], c="red", s=5)
    # line = ax.scatter(list(map(lambda x:-(x-thetas_rad[80])-25/180*cmath.pi,rtheta)), distance2[80:], c="blue", s=5)
    if('line' in locals()):
        line.remove()
    if('line2' in locals()):
        line2.remove()
    # line = ax.scatter(list(map(lambda x:x+(-20)/180*cmath.pi,ltheta)), distance2[:80], c="red", s=5)
    # line2 = ax.scatter(list(map(lambda x:x+(20)/180*cmath.pi,rtheta)), distance2[80:], c="blue", s=5)
    line = ax.scatter(ltheta, distance2[:80], c="red", s=5)
    line2 = ax.scatter(rtheta, distance2[80:], c="blue", s=5)
    ax.set_theta_offset(math.pi / 2)
    ax.set_theta_offset(math.pi / 2)
    plt.pause(0.00001)

    distance.clear()
    distance2.clear()
    
    

# 4+1+1+2+322+1 = 331
# datas = []
# while datas[0:3] !=b'\xa5\xa5\xa5\xa5':
#     datas = ser.read_until(size=331)

# # while packetHeader !=b'\xa5\xa5\xa5\xa5':
# #     packetHeader = ser.read(4)
# # address = ser.read(1)
# # commandType = ser.read(1)
# # dataLength = ser.read(2)
# # checkCode = ser.read(1)

# packetHeaderNumber = datas[0:3]
# addressNumber = int.from_bytes(datas[4],"little")
# commandTypeNumber = int.from_bytes(datas[5],"little")
# dataLengthNumber = int.from_bytes(datas[6:7],"little")

# checkCodeNumber = int.from_bytes(datas[330],"little")

# tmpString = ""
# lines = list()
# angles = list()
# distances = list()

ser.write([0xA5,0xA5,0xA5,0xA5,0x00,0x64,0x00,0x00,0x64])
ser.close()



