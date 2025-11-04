from gpiozero import PWMLED
from time import sleep

motorl = PWMLED(13)
motorr = PWMLED(12)

while True:
    motorl.value = .75
    motorr.value = .3
    sleep(5)
    motorl.value = 0
    motorr.value = 0
