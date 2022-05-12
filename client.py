import math
import numpy as np 
import socket
import pickle

baseWeight=0.6
baseLenght=1

xStart=0
yStart=0
angleStart=0
dt=0.5

def odometry(angularVelocity, linearVelocity, angleCenterMass, xStart, yStart, angleStart, dt):
    Vxr=linearVelocity*math.cos(angleCenterMass)
    Vyr=linearVelocity*math.sin(angleCenterMass)
    xEnd=xStart+dt*(Vxr*math.cos(angleStart)-Vyr*math.sin(angleStart))
    yEnd=yStart+dt*(Vxr*math.sin(angleStart)+Vyr*math.cos(angleStart))
    angleEnd=angleStart+dt*angularVelocity
    xStart=xEnd
    yStart=yEnd
    angleStart=angleEnd
    writeOdometry=open('Odometry.txt', 'a')
    try:
        writeOdometry.write(f'[xEnd={xEnd}, yEnd={yEnd}, angleEnd={angleEnd}]\n [angularVelocity={angularVelocity}, linearVelocity={linearVelocity}, angleCenterMass={angleCenterMass}]\n')
    finally:
        writeOdometry.close()
    return [xEnd, yEnd, angleEnd]


def abFilter(values, xk_1, yk_1):
    alpha=0.355; beta=0.0000005; dt=0.5
    xk=xk_1+(np.array(yk_1)*dt)
    yk=yk_1
    rk=values-xk
    xk=xk+alpha*np.array(rk)
    yk=yk+np.array((beta*np.array(rk)))/dt
    xk_1=xk
    yk_1=yk
    filteredValues=xk_1
    return [filteredValues, xk_1, yk_1]


def ssFilter(sample,k):
    filteredValues=(np.sum(sample, axis=0))/k
    return filteredValues


sock=socket.socket()
sock.connect(('localhost',10000))
sampleAngles=[]
sampleSpeeds=[]
xk_1_angles=np.zeros((4))
yk_1_angles=np.zeros((4))
xk_1_speeds=np.zeros((4))
yk_1_speeds=np.zeros((4))
k=0
while True:
    data=sock.recv(1024)
    if not data:
        break
    data=pickle.loads(data)
    print(f'Получено:{data}')
    angles=np.array(data[0])
    speeds=np.array(data[1])
    k=k+1
    posithionLenght=np.array([baseLenght/2, baseLenght/2, -baseLenght/2, -baseLenght/2])
    positionWeight=np.array([baseWeight/2, -baseWeight/2, baseWeight/2, -baseWeight/2])
    xRotationAverage=0
    yRotationAverage=0
    sampleAngles.append(angles)
    sampleSpeeds.append(speeds)

    # if (k>5):
    #     sampleAngles.pop(0)
    #     sampleSpeeds.pop(0)
    #     angles=ssFilter(sampleAngles,5)
    #     speeds=ssFilter(sampleSpeeds,5)
    # else:
    #     angles=ssFilter(sampleAngles,k)
    #     speeds=ssFilter(sampleSpeeds,k)

    [angles, xk_1_angles, yk_1_angles]=abFilter(angles, xk_1_angles, yk_1_angles)
    [speeds, xk_1_speeds, yk_1_speeds]=abFilter(speeds, xk_1_speeds, yk_1_speeds)

    for i in range(0, angles.size-1):
        for j in range(i+1, angles.size):
            if ((i==0) and (j==3)) or ((i==1) and (j==2)):
                continue
            else:
                b1=positionWeight[i]-math.tan(angles[i]+math.pi/2)*posithionLenght[i]
                b2=positionWeight[j]-math.tan(angles[j]+math.pi/2)*posithionLenght[j]
                xCenterRotation=(b2-b1)/(math.tan(angles[i]+math.pi/2)-math.tan(angles[j]+math.pi/2)) #x=(b2-b1)/(k1-k2)
                xRotationAverage=xRotationAverage+xCenterRotation
                yCenterRotation=math.tan(angles[i]+math.pi/2)*xCenterRotation+b1 #y=k1x+b1
                yRotationAverage=yRotationAverage+yCenterRotation
    xRotationAverage=xRotationAverage/4
    yRotationAverage=yRotationAverage/4

    radius=math.sqrt(xRotationAverage**2+yRotationAverage**2)

    if radius==math.inf:
        angleCenterMass=angles[0]
        linearVelocity=speeds[0]
        angularVelocity=0
    elif radius<1e-6:
        xRotationAverage=0
        yRotationAverage=0
        angleCenterMass=0
        radius=0
        linearVelocity=0
        angularVelocity=speeds[0]/radiusFL
    else:
        if (xRotationAverage > 0) and (yRotationAverage > 0):
            angleCenterMass=-math.atan2(xRotationAverage, yRotationAverage)
        elif (xRotationAverage < 0) and (yRotationAverage > 0):
            angleCenterMass=math.atan2(-xRotationAverage, yRotationAverage)
        elif (xRotationAverage < 0) and (yRotationAverage < 0):
            angleCenterMass=-math.atan2(-xRotationAverage, -yRotationAverage)
            radius=-radius
        elif (xRotationAverage > 0) and (yRotationAverage < 0):
            angleCenterMass=math.atan2(xRotationAverage, -yRotationAverage)
            radius=-radius
        radiusFL=math.sqrt((baseLenght/2-xRotationAverage)**2+(baseWeight/2-yRotationAverage)**2)
        angularVelocity=speeds[0]/radiusFL
        linearVelocity=angularVelocity*radius
    [xEnd, yEnd, angleEnd]=odometry(angularVelocity, linearVelocity, angleCenterMass, xStart, yStart, angleStart, dt)
    data2=[xEnd, yEnd, angleEnd]
    print(f'Отправка:{data2}')
    data2=pickle.dumps(data2)
    sock.send(data2)
sock.close