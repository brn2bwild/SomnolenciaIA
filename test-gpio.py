import RPi.GPIO as gpio
import time

BLUE_LED = 23
RED_LED = 24
BUZZER = 25

gpio.setmode(GPIO.BOARD)

gpio.setup(BLUE_LED, GPIO.OUTPUT)
gpio.setup(RED_LED, GPIO.OUTPUT)
gpio.setup(BUZZER, GPIO.OUTPUT)


gpio.output(BLUE_LED, True)
time.sleep(2)
gpio.output(BLUE_LED, False)
gpio.output(RED_LED, True)
time.sleep(2)
gpio.output(RED_LED, False)
gpio.output(BUZZER, True)
time.sleep(2)
gpio.output(BUZZER, False)
