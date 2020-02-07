#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''

'''

# ----------------------
# importacion de los modulos
# ----------------------
 
import sys, getopt
import cv2
import numpy as np
import os
import threading
import RTIMU
import pygame
import time
import json

# ----------------------
print('Version de Python: ',(".".join(map(str, sys.version_info[:3]))))  
print('Version de OpenCV: ', cv2.__version__)
print('Version de PyGame: ', pygame.__version__)

IMU_estable = False

#Valor de la pocicion original del IMU

IMU_Origen = [0,0,0]

#Valor donde se guarda la pocicion actual del IMU

IMU = (0,0,0)
bloqueo_IMU = threading.Lock()

#Variable global donde se va a guardar la pocicion de la cabeza en la terna roll pich y yaw

Situacion = [0,0,0]
bloqueo_Situacion = threading.Lock()

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

#Creo array de tiempos y archivo

tiempo_inicio = time.perf_counter()
tiempos = {}
contador = 0

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
    global contador
    # greyscale Palette
    grey_palette = [(0, 0, 0)]
    for i in range(1, 256):
        grey_palette.append( (i, i, i) )
    tiempos['time'] = []    
    t0_display = float()
    tf_display = float()
    screen = display_init()
    while True:
        '''En teoria segun la libreria pygame.display.flip() debiera esperar al VSync por estar accediendo al FB
        https://www.pygame.org/docs/ref/display.html#pygame.display.flip'''
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_display'].append(convert)
        t0_display = time.perf_counter()
        jsondata = '{"t0_display":%4.1f}' %((t0_display - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)
        # Limpio el display
        #screen.fill((0,0,0))
        # Copio la variable Frame a una imagen de pygame
        with bloqueo_Frame:
            pgFrame = pygame.image.frombuffer(Frame, (WIDTH,HEIGHT), 'P')
        pgFrame.set_palette(grey_palette)
        #Escribo la pantalla
        screen.blit(pgFrame, (0,0))
        #Cambio el cuadro
        pygame.display.flip()
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_display'].append(convert)
        tf_display = time.perf_counter()
        jsondata = '{"tf_display":%4.1f}' % ((tf_display - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)


def dibujarHorizonte(grados, horizonte_img):
    global Frame
    (h, w) = horizonte_img.shape[:2]
    
    center = (w / 2, h / 2)
    
    M = cv2.getRotationMatrix2D(center, grados, 1)
    rotated90 = cv2.warpAffine(horizonte_img, M, (h, w))
    
    porcion_fondo = Frame[int(WIDTH/2):int((HEIGHT-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))]
    
    #dst = cv2.addWeighted(rotated90,1,porcion_fondo,1,0)
    dst = cv2.bitwise_or(rotated90,porcion_fondo)
    
    Frame[int(WIDTH/2):int((HEIGHT-rotated90.shape[0])),int((WIDTH/2)-(rotated90.shape[0]/2)):int((WIDTH/2)+(rotated90.shape[0]/2))] = dst
    
    return True

def dibujarObjetivos():
    '''no lo tengo claro aun'''
    global Frame
    global Lista_c
    Lista_c.clear()
    
    k = 300 / tan(10) #Pixeles dividido tangente del FOV
    for objetivo in Objetivos:
        objetivo_Proyectado = np.array(objetivo) - Situacion
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
    global retardo_update_frame
    global tiempos
    global contador
    tiempos['time'] = []
    t0_frame = float()
    tf_frame = float()
    #Cargo imágenes
    fondo_img = cv2.imread(os.path.join('/home/pi/fondo2.png'),cv2.IMREAD_GRAYSCALE)
    horizonte_img = cv2.imread(os.path.join('/home/pi/horizonte2.png'),cv2.IMREAD_GRAYSCALE)
    #objetivo_img = cv2.imread(os.path.join('/home/pi/Objetivo.png'),cv2.IMREAD_GRAYSCALE)
    while True:
        '''con los datos de Situacion, objetivos y demas vedura costruyo un nuevo frame.'''
        evento_update_situacion.wait()
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_frame'].append(convert)
        t0_frame = time.perf_counter()
        jsondata = '{"t0_frame":%4.1f}' % ((t0_frame - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)
        font = cv2.FONT_HERSHEY_PLAIN
        with bloqueo_Frame:
            Frame[0:fondo_img.shape[0], 100:100+fondo_img.shape[1]] = fondo_img
            with bloqueo_Situacion:
                cv2.putText(Frame, 'Situacion del casco: roll:%4.1f pich:%4.1f yaw:%4.1f' % tuple(Situacion), 
                            (110,400), font, 0.7, 255, 1, cv2.LINE_AA)
                dibujarHorizonte(Situacion[0], horizonte_img)
                dibujarObjetivos()
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_frame'].append(convert)         
        tf_frame = time.perf_counter()
        jsondata = '{"tf_frame":%4.1f}' % ((tf_frame - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)
        cv2.imwrite('frame.png',Frame)
        evento_update_situacion.clear()

evento_IMU = threading.Event()
def read_IMU():
    """Lee de forma continua el IMU.
    Cuando existe un nuevo dato debe exitar el evento evento_IMU
    Actualiza IMU con un np.array en grados de (roll, pich, yaw)
    Se debe bloquear IMU en la escritura.
    """
    global IMU
    global tiempos
    global contador
    tiempos['time'] = []
    t0_IMU = float()
    tf_IMU = float()
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
    while True:
        time.sleep(poll_interval*1.0/1000.0)
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_IMU'].append(convert)
        t0_IMU = time.perf_counter()
        jsondata = '{"t0_IMU":%4.1f}' % ((t0_IMU - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)
        while imu.IMURead() == False:
            time.sleep(0)
        with bloqueo_IMU:
            IMU = np.degrees(imu.getIMUData()["fusionPose"])
        evento_IMU.set()
        #contador += 1
        #jsondata = '{"muestra":%d}' %contador
        #convert = json.loads(jsondata)
        #tiempos['time_IMU'].append(convert)
        tf_IMU = time.perf_counter()
        jsondata = '{"tf_IMU":%4.1f}' % ((tf_IMU - tiempo_inicio)*1000)
        convert = json.loads(jsondata)
        tiempos['time'].append(convert)
        
    
def estabilidad():
    """Funcion bloqueante
    Almacena hasta 50 muestras y evalua el desvio estandar para verificar si la plataforma inecial esta estabilisada"""
    global IMU_Origen
    global IMU
    global IMU_estable
    #Creo el array donde voy a ir guardando las muestras
    muestras = []

    Muestas_estabibilidad = 50
    while True:
        #print('Esperando el evento_IMU')
        evento_IMU.wait()
        if evento_IMU.is_set():
            evento_IMU.clear()
        #print('Cantidad de muestras almacenadas: ',muestras.__len__())
        #Coloco la ultima lectura del MPU al final del array
        muestras.append(IMU)
        #Pregunto si la cantidad de muestras es mayor a 50
        
        if muestras.__len__() > Muestas_estabibilidad:
            #Calculo la desviacion estandar de cada columna
            DesvioStandar = np.std(muestras, 0)
            Estable = DesvioStandar [0] + DesvioStandar [1] + DesvioStandar [2]
            muestras.pop(0)
            if Estable <= 0.1:
                IMU_estable = True
                IMU_Origen = IMU
                return True
            else:
                IMU_estable = False

evento_update_situacion = threading.Event()
def update_situacion():
    """Funcion que actualiza la pocicion de la cabeza.
    Se debe considerar: la correccion por referencia optica, 
                        el valor actual del IMU,
                        el valor de IMU en el origen,
                        las proyecciones de los giroscopos sobre los verdaderos ejes de rotacion.
    Se debe ejecutar ante los siguientes eventos:
                                Hay un nuevo dato del IMU.
                                Hay un nuevo dato de la referencia Optica.
    Se debe bloquear Actitud al memento de escribirla.
    """
    global Situacion
    global IMU_Origen
    
    while True:
        evento_IMU.wait()
        with bloqueo_Situacion and bloqueo_IMU: #¿Sera real el doble bloqueol?
            Situacion = IMU - IMU_Origen
        evento_update_situacion.set()
        evento_IMU.clear()
'''
def update_refencia_optica:
    cap = cv2.VideoCapture(0)#argumento 0 si leo de la camara web
        
    ####VALORES QUE SE CREE VA A TENER EL LED AL ARRANCAR EL PROGRAMA####
    
    #areas en pixeles
    areaminima=50
    areamaxima=450
    
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
        print('Capturando video para referencia optica de: ')
        print("Error abriendo la transmision de video o archivo")
    
    #Lee y muestra marcos de videos hasta que el video se complete o 
    #el usuario salga presionando la tecla ESC
    while(cap.isOpened()):
        
        # Captura marco por marco 
        ret, frame = cap.read()
        
        #paso de RGB a Escala de Grises
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        #Crear una mascara que filtre el color blanco
        #Elijo los valores maximos y minimos para filtrar el blanco
        
        min_color = 250
        max_color = 255
        
        mask = cv2.inRange(frameGray, min_color, max_color)
        
        #Calculo los contronos
        im2, contornos, jerarquia = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)      
                
        for i in range (0,len(contornos)):#recorro todos los contornos encontrados
        cnt=contornos[i]
        area=cv2.contourArea(cnt)#calculo el area del controno i
        
        if(area>35 and area>areaminima and area<areamaxima and area<450):#filtropor area
            (x, y), radio = cv2.minEnclosingCircle (cnt)#encierro en el minimo circulo posible el contorno
            center = (int (x), int (y))
            radio = int (radio)
            
        #me aseguro que no haya mucha diferencia entre frames contiguos         
        if(abs(ejex-x)<diferenciaposicion and abs(ejey-y)<diferenciaposicion and radio<difereciaradio and radio<20):
            cv2.circle (frame, center, radio, (0,255,0), 2)#senalo con un circulo la posicion rastreada
            print("coordenada x: %d" % x)#Imprimo la coordenada en x
            print("coordenada y: %d" % y)#Imprimo la coordenada en y
        
        areaminima=area-(area*0.18)#establezco una nueva areaminima
        areamaxima=area+(area*0.18)#establezco una nueva areamaxima
        
        ejex=x
        ejey=y
        
        difereciaradio=radio+3#restablezco el radio maximo
        
        break#si ya encontro el que satisface la condicion que lo corte nomas
        else:
        
        contador+=1
                    #si esta mucho tiempo sin rastrear nada
        if (contador>25):
                      #reinicio lo valores
        areaminima=50
        areamaxima=450
        
        ejex=300
        ejey=200
        
        difereciaradio=20
        diferenciaposicion=80
        
        contador=0
        
        if ret == True:
              # Muestra el resultado de los frame
        
        cv2.imshow('Frame',frame)#pongo el que quiero que se vea
        
        
              # Press esc on keyboard to  exit
        if cv2.waitKey(1) & 0xFF == 27: #codigoascii de Esc es 27
        break
        
              # Break the loop
        else:
        break
    
    
    # When everything done, release the video capture object
    cap.release()
    
    # Cerrar todos los frames
    cv2.destroyAllWindows()
'''
def print_telemetria():
    tiempo_0 = float()
    while True:
        os.system('clear')
        print('Situacion del casco: roll:\t%4.1f pich:\t%4.1f yaw:\t%4.1f' % tuple(Situacion)) 
        print('IMU del casco: roll:\t%4.1f pich:\t%4.1f yaw:\t%4.1f' % tuple(IMU)) 
        print('IMU estable:\t',IMU_estable)
        print('Para actualizar Frame se demora %3.0f ms' % (retardo_update_frame*1000))
        print('Se acutulariza Display cada %3.0f ms' % (retardo_update_display*1000))
        print('Coordenadas de Objetivos dibujados',Lista_c)
        time.sleep(0.1)
 
'''Creo los hilos'''
hilo_read_IMU = threading.Thread(name='read_IMU', target=read_IMU)
hilo_print_telemetria = threading.Thread(name='telemetria', target=print_telemetria)
hilo_update_situacion = threading.Thread(name='update_situacion', target=update_situacion)
hilo_update_frame = threading.Thread(name='update_frame', target=update_frame)
hilo_update_display= threading.Thread(name='update_display', target=update_display)

hilo_print_telemetria.setDaemon(True)
hilo_read_IMU.setDaemon(True)
hilo_update_display.setDaemon(True)
hilo_update_frame.setDaemon(True)
hilo_update_situacion.setDaemon(True)



'''Inicio de algo ;-('''
hilo_read_IMU.start()
hilo_update_situacion.start()
hilo_update_frame.start()
hilo_update_display.start()

time.sleep(2)
hilo_print_telemetria.start()
time.sleep(2)
estabilidad()


time.sleep(20)
with open('data2.json', 'w') as file:
    json.dump(tiempos, file, indent=4)