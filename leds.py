import RPi.GPIO as gpio
import time

BLUE_LED = 16
RED_LED = 18
BUZZER = 22

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

gpio.setup(BLUE_LED, gpio.OUT)
gpio.setup(RED_LED, gpio.OUT)
gpio.setup(BUZZER, gpio.OUT)

while True:
    gpio.output(BLUE_LED, True)
    time.sleep(1)
    gpio.output(BLUE_LED, False)
    gpio.output(RED_LED, True)
    time.sleep(1)
    gpio.output(RED_LED, False)
