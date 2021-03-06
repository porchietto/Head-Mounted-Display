#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ----------------------
# importacion de los modulos
# ----------------------

import sys, getopt
import cv2
import numpy as np
import os
import threading
import pygame
import time
from MPU9250Driver import MPU

import json
# ----------------------
print('Version de Python: ',(".".join(map(str, sys.version_info[:3]))))  
print('Version de OpenCV: ', cv2.__version__)
print('Version de PyGame: ', pygame.__version__)

#Valor donde se guarda la pocicion actual del IMU

bloqueo_IMU = threading.Lock()

#Variable global donde se va a guardar la pocicion de la cabeza en la terna roll pich y yaw

Situacion = [0,0,0]
bloqueo_Situacion = threading.Lock()

OffSetGyro = [0,0,0]

# -------------------
# constantes
# ------------------

WIDTH = 800
HEIGHT = 600

OffSetX = 100
OffSetY = 0

#Configuro variable global para hacer de interface a frame

Frame = np.zeros((HEIGHT,WIDTH),dtype=np.uint8)
bloqueo_Frame = threading.Lock()

#Variable Global para hacer de interface para la telemetria del video que capturan las webcam a una terminal remota

Frame_str = []
bloqueo_Frame_str = threading.Lock()

#Creo array de tiempos

tiempo_inicio = time.perf_counter()
tiempos = []
contador_frame = 0


'''Redefinicion de funciones matenaticas para usarlas en grados.
Es deceable poder implementar en futuro optimizaciones como una serie de lorenz para mejorar la velocidad.'''
def sin(grados):
    return np.sin(np.radians(grados))

def cos(grados):
    return np.cos(np.radians(grados))

def tan(grados):
    return np.tan(np.radians(grados))

'''Lista de tuplas que contiene todos los objetivos a marcar.
   Estan expresado en gados. Roll, Pitch y Yaw repectivamete.'''   
Objetivos = [(0,0,0), (0,0,5), (0,0,15), (0,0,20), (0,0,25), (0,0,30), (0,0,35)]

'''Lista de objetivos dibujados solo para propocitos de debub'''
Lista_c = []

def display_init():
    '''Funcion para Configurar e iniciar la pantalla'''
    pygame.display.init()
    screen = pygame.display.set_mode((WIDTH,HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    print(pygame.display.Info())
    return screen

retardo_update_display = float()  
def update_display():
    """Trasfomo Frame en un objeto de pygame y lo escribo en el FrameBuffer
    utilizar un semaforo para bloquear a Frame mientras se lee"""
    global retardo_update_display
    global bloqueo_Frame
    global tiempos
    global contador_frame
    # greyscale Palette
    grey_palette = [(0, 0, 0)]
    for i in range(1, 256):
        grey_palette.append( (i, i, i) )
    screen = display_init()
    while True:
        '''En teoria segun la libreria pygame.display.flip() debiera esperar al VSync por estar accediendo al FB
        https://www.pygame.org/docs/ref/display.html#pygame.display.flip'''
        contador_frame += 1
        # Limpio el display
        #screen.fill((0,0,0))
        # Copio la variable Frame a una imagen de pygame
        t0_frame = time.perf_counter()
        with bloqueo_Frame:
            t_escritura = time.perf_counter()
            pgFrame = pygame.image.frombuffer(Frame, (WIDTH,HEIGHT), 'P')
        # Tiempo en el cual se recupera el frame y guarda en una variable de pygame
        tf_frame = time.perf_counter()
        t_frame = 't_frame', round(((tf_frame - tiempo_inicio)*1000),2), contador_frame
        tiempos.append(t_frame)
        # Retardo para recuperar el frame
        retardo_frame = 'retardo_frame', round(((tf_frame - t0_frame)*1000),2), contador_frame
        tiempos.append(retardo_frame)
        # Retardo para escribir el frame en una variable de pygame
        retardo_escritura = 'retardo_escritura',round(((tf_frame - t_escritura)*1000),2), contador_frame
        tiempos.append(retardo_escritura)
        t0_display = time.perf_counter()
        pgFrame.set_palette(grey_palette)
        #Escribo la pantalla
        screen.blit(pgFrame, (0,0))
        #Cambio el cuadro
        t0_flip = time.perf_counter()
        pygame.display.flip()
        tf_flip = time.perf_counter()
        # Retardo del flip
        retardo_flip = 'retardo_flip',round(((tf_flip - t0_flip)*1000),2), contador_frame
        tiempos.append(retardo_flip)
        # Tiempo en el cual se actualiza la pantalla y comienza el bucle nuevamente
        tf_display = time.perf_counter()
        t_display = 't_display', round(((tf_display - tiempo_inicio)*1000),2), contador_frame
        tiempos.append(t_display)
        # Retardo para actualizar la pantalla despues que se obtiene el frame
        retardo_display = 'retardo_display', round(((tf_display - t0_display)*1000),2), contador_frame
        tiempos.append(retardo_display)
        # Retardo total del bucle
        retardo_bucle = 'retardo_bucle_display', round(((tf_display - t0_frame)*1000),2), contador_frame
        tiempos.append(retardo_bucle)


def dibujarHorizonte(grados, horizonte_img):
    global Frame
    (h, w) = horizonte_img.shape[:2]
    
    center = (w / 2, h / 2)
    
    M = cv2.getRotationMatrix2D(center, grados, 1)
    rotated90 = cv2.warpAffine(horizonte_img, M, (h, w))
    
    porcion_fondo = Frame[int(WIDTH/2):int((HEIGHT-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))]
    
    dst = cv2.addWeighted(rotated90,1,porcion_fondo,1,0)
    
    Frame[int(WIDTH/2):int((HEIGHT-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))] = dst
    
    return True

def dibujarObjetivos(Situacion_local):
    '''no lo tengo claro aun'''
    global Frame
    global Lista_c
    Lista_c.clear()
    
    k = 300 / tan(10) #Pixeles dividido tangente del FOV
    for objetivo in Objetivos:
        objetivo_Proyectado = np.array(objetivo) - Situacion_local
        if np.all(np.abs(objetivo_Proyectado)[1:] < [10,10]):
            '''Si el objetivo esta en el campo de vision'''
            c = tuple(np.round(tan(objetivo_Proyectado[1:]) * k + np.array(Frame.shape)/2)[::-1].astype(int))
            #[1:] me quedo con los elemntos apartir de 1, [::-1] invierto el orden de los elemntos
            cv2.drawMarker(Frame, c, 255, cv2.MARKER_TRIANGLE_UP, 25, 3)
            Lista_c.append(c)
            
retardo_update_frame = float()  
def update_frame():
    """Construye un nuevo Frame.
    Se debe ejecura ante los siguientes eventos:
                                Existe una nueva actitud.
                                Se cambio o agrego un objetivo.
                                Se cambio o agrego informacion de pantalla.
    Se debe bloquear Frame mientras se los escribe.
    horizonte: grados de la cabeza respecto al horizonte.
    objetivos: Lista de tuplas en theta, phi de cada uno de los objetivos respecto de la cabeza."""
    global Frame
    global bloqueo_Frame
    global retardo_update_frame
    #global contador_frame
    #tiempo_0 = float()
    #Cargo imágenes
    fondo_img = cv2.imread(os.path.join('/home/pi/fondo2.png'),cv2.IMREAD_GRAYSCALE)
    horizonte_img = cv2.imread(os.path.join('/home/pi/horizonte2.png'),cv2.IMREAD_GRAYSCALE)
    #objetivo_img = cv2.imread(os.path.join('/home/pi/Objetivo.png'),cv2.IMREAD_GRAYSCALE)
    font = cv2.FONT_ITALIC
    while True:
        '''con los datos de Situacion, objetivos y demas vedura costruyo un nuevo frame.'''
        t0_bloqueo = time.perf_counter()
        with bloqueo_Frame:
            # Tiempo en el cual se bloquea el frame
            tf_bloqueo = time.perf_counter()
            t_bloqueo = 't_bloqueo', round(((tf_bloqueo - tiempo_inicio)*1000),2), contador_frame
            tiempos.append(t_bloqueo)
            # Retardo para el cual el frame esta bloqueado
            retardo_bloqueo = 'retardo_bloqueo', round(((tf_bloqueo - t0_bloqueo)*1000),2), contador_frame
            tiempos.append(retardo_bloqueo)
            t0_dibujo = time.perf_counter()
            Frame[0:fondo_img.shape[0], 100:100+fondo_img.shape[1]] = fondo_img
            with bloqueo_Situacion:
                Situacion_local = Situacion
            cv2.putText(Frame, 'Situacion del casco: roll:%4.1f pich:%4.1f yaw:%4.1f' % tuple(Situacion_local), 
                        (110,400), font, 0.5, 255, 1, cv2.LINE_AA)
            dibujarHorizonte(Situacion_local[0], horizonte_img)
            dibujarObjetivos(Situacion_local)
        # Tiempo en el cual se termina de dibujar el frame
        tf_dibujo = time.perf_counter()
        t_dibujo = 't_dibujo', round(((tf_dibujo - tiempo_inicio)*1000),2), contador_frame
        tiempos.append(t_dibujo)
        # Retardo para dibujar el frame
        retardo_dibujo = 'retardo_dibujo', round(((tf_dibujo - t0_dibujo)*1000),2), contador_frame
        tiempos.append(retardo_dibujo)
        if retardo_update_frame < 15/1000:
            time.sleep(15/1000-retardo_update_frame)
        # Retardo total del bucle
        retardo_bucle = 'retardo_bucle_frame', round(((tf_dibujo - t0_bloqueo)*1000),2), contador_frame
        tiempos.append(retardo_bucle)
        
evento_IMU = threading.Event()
def read_IMU():
    """Lee de forma continua el IMU.
    Cuando existe un nuevo dato debe exitar el evento evento_IMU
    Actualiza IMU con un np.array en grados de (roll, pich, yaw)
    Se debe bloquear IMU en la escritura.
    """
    global Situacion
    gyro = 500      # 250, 500, 1000, 2000 [deg/s]
    acc = 4         # 2, 4, 7, 16 [g]
    mag = 16        # 14, 16 [bit]
    tau = 0.98
    global mpu 
    mpu = MPU(gyro, acc, mag, tau)
    global tiempos

    # Set up the IMU and mag sensors
    mpu.setUpIMU()
    mpu.setUpMAG()
    
    # Calibrate the mag or provide values that have been verified with the visualizer
    #mpu.calibrateMagGuide()
    bias = [145, 145, -155]
    scale = [1.10, 1.05, 1.05]
    #mpu.setMagCalibration(bias, scale)

    # Calibrate the gyro with N points
    mpu.calibrateGyro(100)
    
    while True:
        mpu.integartorFilter()
        with bloqueo_Situacion:
            Situacion = np.array((mpu.gyroRoll, mpu.gyroPitch, mpu.gyroYaw)) - OffSetGyro

retardo_update_refencia_optica = float()
def update_refencia_optica():
    tiempo_0 = float()
    global retardo_update_refencia_optica
    global Frame_str
    global OffSetGyro
    '''Contro de WEBCam'''
    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)#0 corresponde a /dev/video0, backend V4L2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    #cap.set(cv2.CAP_PROP_FPS, 5)
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    Frame_str = np.zeros((int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),3),dtype=np.uint8)

    ####VALORES QUE SE CREE VA A TENER EL LED AL ARRANCAR EL PROGRAMA####
    
    #areas en pixeles
    areaminima=cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 11000
    areamaxima=cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 7000
    
    #coordenadas en X e Y
    ejex=300
    ejey=200
    
    #radio maximo
    difereciaradio=20
    
    #diferencia maxima de posicion en frames contiguos
    diferenciaposicion=80
    
    contador=0
    #########################################################
    
    if (cap.isOpened()): #devuelve verdadero si la captura del video ya se ha inicializado
        print('Capturando video para referencia optica de: camara por defecto')
    else:
        print("Error abriendo la captura de video")
        exit()
  
    cap.set(cv2.CAP_PROP_CONTRAST, 128)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
    cap.set(cv2.CAP_PROP_ZOOM, 0)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
    cap.set(cv2.CAP_PROP_FOCUS, 30) # 0 cerca 254 lejos
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # 1 desabilitado 3 habilitado
    cap.set(cv2.CAP_PROP_EXPOSURE, 40) # 0 corto tiempo de expocicion 2048 largo tiempo de expocicion
    cap.set(cv2.CAP_PROP_AUTO_WB, 1) # 0 Deshabilita el balance de blancos
    #cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 5000)
    cap.set(cv2.CAP_PROP_GAIN,0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    
    V = 145
    while(True):
        '''PROP_CAP = cv2.CAP_PROP_BRIGHTNESS
        if cap.get(PROP_CAP) < 254:
            cap.set(PROP_CAP, cap.get(PROP_CAP) + 1)
        else:
            cap.set(PROP_CAP, 1)
        print(cap.get(PROP_CAP))
        Ver el link 
        https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
        '''
        umbral_bajo = (155,1,250)
        umbral_alto = (165,3,255)
        '''if V < 170:
            V += 0.1
        else:
            V = 145
        
        umbral_bajo = (int(V),1,250)
        umbral_alto = (int(V) +10,3,255)
        print(umbral_bajo)'''
        
        tiempo_0 = time.perf_counter()
        cap.grab() #retorna un bool se puedu usar para controlar el while
        '''aqui le ordena a la camara capturar la imagen,
        se va a demorar hasta que este capturada  si la camara ya la capturo lo devuelve de imediato , 
        lo importante es que la marca de tiempo al salir del cuadro es precisa
        https://medium.com/@gastonace1/detecci%C3%B3n-de-objetos-por-colores-en-im%C3%A1genes-con-python-y-opencv-c8d9b6768ff
        
        '''
        
        # Captura marco por marco 
        ret, frame = cap.retrieve()
        '''Aqui recibe la imagen desde la camara, es un cuadro que ya se capturo previamente y quedo en el buffer'''
        if ret is True:
           pass#Frame_str = frame#para enviar la telemetria de video
        '''El espacio de color HSV trabaja por tonalidad, saturacion y brillo, 
        lo que es muy util ya que la saturacion y brillo pueden cambiar deacuerdo a las condiciones externas 
        pero la tonalidad no'''
            
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_mask = cv2.inRange(frame_hsv, umbral_bajo, umbral_alto)
        frame_gauss = cv2.GaussianBlur(frame_mask,(9,9),0)
        try:
            Frame_str = frame#cv2.cvtColor(frame_gauss, cv2.COLOR_GRAY2RGB)
        except:
            pass
        #Calculo los contronos
        contornos, jerarquia = cv2.findContours(frame_gauss,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        try:
            cv2.drawContours(Frame_str, contornos, -1, (0,255,0), 3)
        except:
            pass
        
        referencias_LEDx = []
        referencias_LEDy = []

        #print(contornos.__len__())      
        for contorno in contornos:#recorro todos los contornos encontrados
            area=cv2.contourArea(contorno)#calculo el area del controno i
            if(areaminima <area< areamaxima):#filtropor area
                (x, y), radio = cv2.minEnclosingCircle (contorno)#encierro en el minimo circulo posible el contorno
                centro = (int (x), int (y))
                radio = int (radio)
                cv2.drawMarker(Frame_str, centro, 255, cv2.MARKER_DIAMOND, 50, 2)
                referencias_LEDx.append(x)
                referencias_LEDy.append(y)
                '''print(centro)
                print(area)
                print(areamaxima)
                print(areaminima)'''
        print(referencias_LEDx.__len__())
        if True:
            try:
                centro = (int(np.mean(referencias_LEDx)), int(np.mean(referencias_LEDy)))
                cv2.drawMarker(Frame_str, centro, 255, cv2.MARKER_STAR, 30, 2)
            except:
                pass
                
        
        OffSetGyro = (mpu.gyroRoll,mpu.gyroPitch,mpu.gyroYaw) - np.array((0,0,0))
        retardo_update_refencia_optica =  time.perf_counter() - tiempo_0

def telemetria_consola():
    while True:
        os.system('clear')
        print('Situacion del casco: roll:\t%4.1f pich:\t%4.1f yaw:\t%4.1f' % tuple(Situacion)) 
        print('IMU del casco: roll:\t%4.1f pich:\t%4.1f yaw:\t%4.1f' % (mpu.gyroRoll,mpu.gyroPitch,mpu.gyroYaw)) 
        print('Para actualizar Frame se demora %3.0f ms' % (retardo_update_frame*1000))
        print('Se acutulariza Display cada %3.0f ms' % (retardo_update_display*1000))
        print('Se capturan imagenes para referencia optica cada %3.0f ms' % (retardo_update_refencia_optica*1000))
        print('Coordenadas de Objetivos dibujados',Lista_c)
        time.sleep(1)

def telemetria_video():
    '''
    mjpeg codec 
    Tranmisor:
    sudo gst-launch-1.0 v4l2src -v  ! video/x-raw,width=1920,height=1080,format=BGR ! videoconvert ! video/x-raw,format=YUY2 ! jpegenc idct-method=ifast ! queue ! rtpjpegpay pt=127 ! udpsink host=192.168.16.15 port=5000
    
    Reseptor :
    gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp,media=(string)video,encoding-name=(string)JPEG" ! rtpjpegdepay ! jpegdec ! xvimagesink sync=false
    '''
    gst_str_rtp = 'appsrc ! videoconvert ! video/x-raw,format=YUY2 ! queue ! jpegenc ! jpegparse ! rtpjpegpay ! udpsink host=192.168.16.15 port=5000'
    fps = 10 #VideoWriter matiene la tasa 
    out = cv2.VideoWriter(gst_str_rtp, cv2.CAP_GSTREAMER, 0, fps, (Frame_str.shape[1], Frame_str.shape[0]), True)
    while True:
        out.write(Frame_str)
 
'''Creo los hilos'''
hilo_read_IMU = threading.Thread(name='read_IMU', target=read_IMU)
hilo_telemetria_consola = threading.Thread(name='telemetria', target=telemetria_consola)
hilo_telemetria_video = threading.Thread(name='telemetria_video', target=telemetria_video)
hilo_update_frame = threading.Thread(name='update_frame', target=update_frame)
hilo_update_display = threading.Thread(name='update_display', target=update_display)
hilo_update_refencia_optica = threading.Thread(name='update_refencia_optica', target=update_refencia_optica)

hilo_telemetria_consola.setDaemon(True)
hilo_telemetria_video.setDaemon(True)
hilo_read_IMU.setDaemon(True)
hilo_update_display.setDaemon(True)
hilo_update_frame.setDaemon(True)
hilo_update_refencia_optica.setDaemon(True)

'''Inicio de algo ;-('''
hilo_read_IMU.start()
time.sleep(1)
hilo_update_frame.start()
hilo_update_display.start()
hilo_update_refencia_optica.start()

time.sleep(5)
hilo_telemetria_consola.start()
hilo_telemetria_video.start()
time.sleep(10)

time.sleep(20)
with open('data8.json', 'w') as file:
    json.dump(tiempos, file, indent=4)
