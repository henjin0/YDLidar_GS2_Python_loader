import cmath
import struct

class result:
    packetHeaderNumber:str
    addressNumber:str
    comemandTypeNumber:str
    dataLengthNumber:int
    envlm:int
    checkCodeNumber:str
    distance:list[float]
    thetas:list[float]

    def __init__(self,packetHeaderNumber,addressNumber,comemandTypeNumber,\
        dataLengthNumber,envlm,checkCodeNumber,distance,thetas):
        self.packetHeaderNumber = packetHeaderNumber
        self.addressNumber = addressNumber
        self.comemandTypeNumber = comemandTypeNumber
        self.dataLengthNumber = dataLengthNumber
        self.envlm = envlm
        self.checkCodeNumber = checkCodeNumber
        self.distance = distance
        self.thetas = thetas

class calclidar:
    # init
    d_compensateK0:float
    d_compensateB0:float
    d_compensateK1:float
    d_compensateB1:float
    bias:float

    angle_p_x :float
    angle_p_y :float
    angle_p_angle :float

    thetas_deg =[] 
    thetas_rad =[]

    def __init__(self,K0,B0,K1,B1,rowBias):
        self.d_compensateK0 = K0/10000.0
        self.d_compensateB0 = B0/10000.0
        self.d_compensateK1 = K1/10000.0
        self.d_compensateB1 = B1/10000.0
        self.bias = float(rowBias)/10

        self.angle_p_x =  1.22
        self.angle_p_y = 5.315
        self.angle_p_angle = 22.5

        for i in range(160):
            if i < 80:
                self.thetas_deg.append(self.leftCamThetaCalc(i))
                self.thetas_rad.append(self.leftCamThetaCalc(i)/180*cmath.pi) 
            else:
                self.thetas_deg.append(self.rightCamThetaCalc(i))
                self.thetas_rad.append(self.rightCamThetaCalc(i)/180*cmath.pi) 

    def leftCamThetaCalc(self,measurement_point):
        measurement_point = 80-measurement_point
        if self.d_compensateB0 > 1:
            tempTheta = self.d_compensateK0 * measurement_point - self.d_compensateB0
        else:
            tempTheta = cmath.atan(self.d_compensateK0 * measurement_point - self.d_compensateB0).real * 180 / cmath.pi
        
        return tempTheta

    def rightCamThetaCalc(self, measurement_point):
        measurement_point = 160 - measurement_point 
        if self.d_compensateB1 > 1:
            tempTheta = self.d_compensateK1 * measurement_point - self.d_compensateB1
        else:
            tempTheta = cmath.atan(self.d_compensateK1 * measurement_point - self.d_compensateB1).real * 180 / cmath.pi
        
        return tempTheta

    def leftCalc(self,tempDist_,theta_index):
        tempDist = (tempDist_ - self.angle_p_x) / cmath.cos((self.angle_p_angle + self.bias - (self.thetas_rad[theta_index]*180/cmath.pi)) * cmath.pi / 180).real; 
            
        tempX_ = cmath.cos((self.angle_p_angle + self.bias) * cmath.pi/ 180).real * tempDist * cmath.cos(self.thetas_rad[theta_index]).real + \
                cmath.sin((self.angle_p_angle + self.bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(self.thetas_rad[theta_index]).real)
        tempY_ = -cmath.sin((self.angle_p_angle + self.bias) * cmath.pi / 180).real * tempDist * cmath.cos(self.thetas_rad[theta_index]).real +\
                cmath.cos((self.angle_p_angle+ self.bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(self.thetas_rad[theta_index]).real)
        tempX = tempX_ + self.angle_p_x
        tempY = tempY_ - self.angle_p_y
        if(tempX != 0.0):
            Dist = cmath.sqrt(tempX * tempX + tempY * tempY).real
            theta = cmath.atan(tempY / tempX).real
        else:
            Dist = 0
            theta = 0
        return Dist,theta

    def rightCalc(self, tempDist_, theta_index):
        tempDist = (tempDist_ - self.angle_p_x) / cmath.cos((self.angle_p_angle  + self.bias + (self.thetas_rad[theta_index]*180/cmath.pi)) * cmath.pi / 180).real; 
            
        tempX_ = cmath.cos(-(self.angle_p_angle + self.bias) * cmath.pi/ 180).real * tempDist * cmath.cos(self.thetas_rad[theta_index]).real + \
                cmath.sin(-(self.angle_p_angle + self.bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(self.thetas_rad[theta_index]).real)
        tempY_ = -cmath.sin(-(self.angle_p_angle + self.bias) * cmath.pi / 180).real * tempDist * cmath.cos(self.thetas_rad[theta_index]).real +\
                cmath.cos(-(self.angle_p_angle+ self.bias) *   cmath.pi / 180).real * (tempDist *  cmath.sin(self.thetas_rad[theta_index]).real)
        tempX = tempX_ + self.angle_p_x
        tempY = tempY_ + self.angle_p_y
        if(tempX != 0.0):
            Dist = cmath.sqrt(tempX * tempX + tempY * tempY).real
            theta = cmath.atan(tempY / tempX).real
        else:
            Dist = 0
            theta = 0
        return Dist,theta

    def dataCheck(self,datas):
        if(len(datas)!=331):
            return False

        if(int.from_bytes(datas[0:4],"little") != 0xa5a5a5a5):
            return False
        
        if(datas[330] != sum(datas[4:330]) & 0x00FF):
            return False
        
        return True
    
    def receiveDataCalc(self,datas):
        packetHeaderNumber = hex(int.from_bytes(datas[0:4],"little"))
        addressNumber = hex(datas[4])
        comemandTypeNumber = hex(datas[5])
        dataLengthNumber = int(datas[7]<<8 | datas[6])
        envlm = int(datas[9]<<8 | datas[8])
        checkCodeNumber = hex(datas[330])

        distance = []
        thetas = []
        for i in range(int(len(datas[10:-1])/2)):
            tempDist = (datas[8+2*i+1]<<8 | datas[8+2*i]) & 0x01ff
            if i < 80:
                Dist,theta = self.leftCalc(tempDist,i)
            else:
                Dist,theta = self.rightCalc(tempDist,i)

            distance.append(Dist)
            thetas.append(theta)
        
        rt = result(packetHeaderNumber,addressNumber,\
                comemandTypeNumber,dataLengthNumber,\
                envlm, checkCodeNumber,\
                distance, thetas)

        return rt


        
