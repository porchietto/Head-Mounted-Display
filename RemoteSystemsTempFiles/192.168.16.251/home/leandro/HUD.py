#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ----------------------
# importacion de los modulos
# ----------------------

import sys, getopt

sys.path.append('.')
import pygame
import math
import os
import RTIMU
import time
import numpy
import cv2

# ----------------------

SETTINGS_FILE = "RTIMULib"

# Inicializacion del MPU

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# Configuracion de lo parámetros

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(False)
imu.setCompassEnable(False)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

# -------------------
# constantes
# ------------------

WIDTH = 800
HEIGHT = 600

OffSetX = 100
OffSetY = 0

Muestas_estabibilidad = 50

#Configuro pantalla

os.putenv('SDL_VIDEODRIVER', 'fbcon')
os.getenv('SDL_VIDEODRIVER')
pygame.display.init()
screen = pygame.display.set_mode((WIDTH,HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
print (("Framebuffer size: %d x %d") % (size[0], size[1]))

#Cargo imagenes
center_fondo = pygame.image.load(os.path.join('/home/leandro/fondo1.png'))
center_horizonte = pygame.image.load(os.path.join('/home/leandro/horizonte.png'))
#center_objetivo = pygame.image.load(os.path.join('/home/leandro/Objetivo.png'))

# Actualiza el display 
def update(X_Objetivo,Y_Objetivo,Angle_Horizonte):
    # Limpio el display
    screen.fill((0,0,0))
    #Escribo el fondo
    screen.blit(center_fondo, (OffSetX, OffSetY))
    # Escribo y roto horizonte
    rotated = pygame.transform.rotate(center_horizonte, Angle_Horizonte)
    screen.blit(rotated, (OffSetX + center_fondo.get_width()/2 - rotated.get_width()/2 ,
        OffSetY + center_fondo.get_height()/2 - center_horizonte.get_height()/2))
    #Escribo el objetivo Desplazado
    #screen.blit(center_objetivo, (X_Objetivo + OffSetX + center_fondo.get_width()/2 - center_objetivo.get_width()/2,
    #    Y_Objetivo + OffSetY + center_fondo.get_height()/2 - center_objetivo.get_height()/2))
    #Cambio el cuadro
    pygame.display.flip()

def estabilidad():
   muestras = []
   while True:
      while imu.IMURead() == False:
         muestras.append(imu.getIMUData()["fusionPose"])
         if muestras.__len__() > 50:
            estab_roll = st.stdev(numpy.transpose(muestras[0]))
            estab_pitch =  st.stdev(numpy.transpose(muestras[1]))
            estab_yaw = st.stdev(numpy.transpose(muestras[2]))
            DesvioStandar = estab_roll + estab_pitch + estab_yaw
            muestras.pop(0)
            if DesvioStandar <= 1:
               print("IMU estable")
               return True
            else:
               print(DesvioStandar)

X_Objetivo=0
Y_Objetivo=0
Angle_Horizonte=0

# set some gyro rate here to test (units: rads/s)

gx = 0.0
gy = 0.0
gz = 0.1

# set accel to indicate horizontal (units: g)

ax = 0.0
ay = 0.0
az = 1.0

# set mag to whatever (or leave as 0 if turned off) (units: uT)

mx = 0.0
my = 0.0
mz = 0.0

imu.setExtIMUData(gx, gy, gz, ax, ay, az, mx, my, mz, 0)

#estabilidad() #Condicion de estabilidad bloqueante.

while True:
   while imu.IMURead() == False:
      fusionPose = imu.getIMUData()["fusionPose"]
      print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),
      	math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
      update(X_Objetivo, Y_Objetivo, math.degrees(fusionPose[0]))
      time.sleep(poll_interval*1.0/1000.0)
