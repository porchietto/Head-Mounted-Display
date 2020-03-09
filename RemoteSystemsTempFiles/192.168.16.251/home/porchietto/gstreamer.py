import time
import cv2
import os
import threading

# Cam properties
fps = 2.
frame_width = 1920
frame_height = 1080
# Create capture
'''https://docs.opencv.org/4.1.2/d4/d15/group__videoio__flags__base.html 
cv2.CAP_V4L2 = 200
cv2.CAP_GSTREAMER = 1800
codigo fuente de VideoCapture() https://github.com/opencv/opencv/blob/master/modules/videoio/src/cap_v4l.cpp
   
cap.get(cv2.CAP_PROP_POS_MSEC)
Returns the millisecond timestamp of the last frame grabbed or 0 if no frames have been grabbed

mjpeg codec
Tranmisor:
sudo gst-launch-1.0 v4l2src -v  ! video/x-raw,width=1920,height=1080,format=BGR ! videoconvert ! video/x-raw,format=YUY2 ! jpegenc idct-method=ifast ! queue ! rtpjpegpay pt=127 ! udpsink host=192.168.16.15 port=5000

Reseptor :
gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp,media=(string)video,encoding-name=(string)JPEG" ! rtpjpegdepay ! jpegdec ! xvimagesink sync=false

sudo v4l2-ctl -d 0 --list-formats-ext
'''
cap = cv2.VideoCapture(0,cv2.CAP_V4L2)

print(cap.get(cv2.CAP_PROP_BRIGHTNESS))
print(cap.get(cv2.CAP_PROP_CONTRAST))
print(cap.get(cv2.CAP_PROP_SATURATION))
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(cap.get(cv2.CAP_PROP_FPS))
# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
#cap.set(cv2.CAP_PROP_FPS, fps)

print(cap.get(cv2.CAP_PROP_BRIGHTNESS))
print(cap.get(cv2.CAP_PROP_CONTRAST))
print(cap.get(cv2.CAP_PROP_SATURATION))
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(cap.get(cv2.CAP_PROP_FPS))
print(cv2.__version__)

fps = cap.get(cv2.CAP_PROP_FPS)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

def decode_fourcc(fourcc):
    """Decodes the fourcc value to get the four chars identifying it

    """
    fourcc_int = int(fourcc)

    # We print the int value of fourcc
    print("int value of fourcc: '{}'".format(fourcc_int))

    # We can also perform this in one line:
    # return "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])

    fourcc_decode = ""
    for i in range(4):
        int_value = fourcc_int >> 8 * i & 0xFF
        print("int_value: '{}'".format(int_value))
        fourcc_decode += chr(int_value)
    return fourcc_decode



# Define the gstreamer sink
#Fuinciona pero demanda mucho CPU #gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=192.168.16.15 port=5000"

gst_str_rtp = 'appsrc ! videoconvert ! video/x-raw,format=YUY2 ! queue ! jpegenc ! jpegparse ! rtpjpegpay ! udpsink host=192.168.16.15 port=5000'

# Check if cap is open
if cap.isOpened() is not True:
    print('Cannot open camera. Exiting.')
    quit()

fourcc = cap.get(cv2.CAP_PROP_FOURCC)
#print(decode_fourcc(fourcc))
# Create videowriter as a SHM sink
out = cv2.VideoWriter(gst_str_rtp, cv2.CAP_GSTREAMER, 0, fps, (frame_width, frame_height), True)


cap.set(cv2.CAP_PROP_CONTRAST, 128)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
cap.set(cv2.CAP_PROP_SATURATION, 128)
cap.set(cv2.CAP_PROP_ZOOM, 0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 30) # 0 cerca 254 lejos
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # 1 desabilitado 3 habilitado
cap.set(cv2.CAP_PROP_EXPOSURE, 30) # 0 corto tiempo de expocicion 2048 largo tiempo de expocicion
cap.set(cv2.CAP_PROP_AUTO_WB, 0) # 0 Deshabilita el balance de blancos
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 5000)
cap.set(cv2.CAP_PROP_GAIN,128)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Loop it
retardo_update_frame = float()  

while True:
    os.system('clear')
    tiempo_0 = time.perf_counter()
    # Get the frame 
    print('grab')
    cap.grab()
    '''aqui le ordena a la camara capturar la imagen,
    se va a demorar hasta que este capturada  si la camara ya la capturo lo devuelve de imediato , 
    lo importante es que la marca de tiempo al salir del cuadro es precisa
    '''
    tiempo_1 = time.perf_counter()
    print('retrive')
    ret, frame = cap.retrieve()
    '''Aqui recibe la imagen desde la camara, es un cuadro que ya se capturo previamente y quedo en el buffer'''
    tiempo_2 = time.perf_counter()
    #time.sleep(50/1000)
    tiempo_3 = time.perf_counter()
    # Check    
    if ret is True:
        # Write to
        out.write(frame)
        tiempo_4 = time.perf_counter()
    else:
        print('Camera error.')
        time.sleep(10)
    '''PROP_CAP = cv2.CAP_PROP_BUFFERSIZE
    if cap.get(PROP_CAP) < 254:
        cap.set(PROP_CAP, cap.get(PROP_CAP) + 1)
    else:
        cap.set(PROP_CAP, 1)
    '''
    print(cap.get(cv2.CAP_PROP_BRIGHTNESS))
    print(cap.get(cv2.CAP_PROP_CONTRAST))
    print(cap.get(cv2.CAP_PROP_SATURATION))
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(cap.get(cv2.CAP_PROP_FPS))
    print('Tiempo de captura de imagen: %3.0f ms' % ((tiempo_1 - tiempo_0)*1000))
    print('Tiempo para recibir al imagen %3.0f ms' % ((tiempo_2 - tiempo_1)*1000))
    print('Tiempo muerto util para otros calculos %3.0f ms' % ((tiempo_3 - tiempo_2)*1000))
    print('Tiempo para enviar cuadro por gstreamer %3.0f ms' % ((tiempo_4 - tiempo_3)*1000))
    print('Tiempo total %3.0f ms' % ((time.perf_counter() - tiempo_0)*1000))
    print('FPS:', 1/(time.perf_counter() - tiempo_0))

hilo_camara = threading.Thread(name='camara', target=camara)
hilo_camara.start()
time.sleep(10)
cap.release()