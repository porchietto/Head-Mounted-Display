import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)

t0 = time.perf_counter()
while (True):
    while ((time.perf_counter() - t0) <= 1/1000):
        pass
    t0 = time.perf_counter()
    GPIO.output(7, GPIO.HIGH)
    time.sleep(200/1000000)
    GPIO.output(7, GPIO.LOW)
