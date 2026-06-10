import board
import neopixel
import time
import digitalio

try:
    pixels = neopixel.NeoPixel(board.D18, 1)
    pixels.fill((255,0,0))
    time.sleep(2)
    pixels.fill((0,255,0))
    time.sleep(2)
    pixels.fill((0,0,0))
except Exception as e:
    print (f"Error : {e}")

try:
    buzzer = digitalio.DigitalInOut(board.D24)
    buzzer.direction = digitalio.Direction.OUTPUT
    buzzer.value = True
except Exception as e:
    print (f"Eror {e}")




#pixels = neopixel.NeoPixel(board.D18, 1)

#pixels.fill((255, 0, 255))

#time.sleep(1)
#pixels.fill((0,255,0))
#time.sleep(1)
#pixels.fill((0,0,0))
#pixels.show()
