from machine import Pin
import time


encoderA_A = Pin(10, Pin.IN)
encoderA_B = Pin(11, Pin.IN)
encoderB_A = Pin(12, Pin.IN)
encoderB_B = Pin(13, Pin.IN)


countA = 0
countB = 0


def encoderA_callback(pin):
    global countA
    if encoderA_A.value() == encoderA_B.value():
        countA += 1
    else:
        countA -= 1

def encoderB_callback(pin):
    global countB
    if encoderB_A.value() == encoderB_B.value():
        countB += 1
    else:
        countB -= 1


encoderA_A.irq(trigger=Pin.IRQ_RISING, handler=encoderA_callback)
encoderB_A.irq(trigger=Pin.IRQ_RISING, handler=encoderB_callback)


try:
    while True:
        print("Encoder A Count:", countA)
        print("Encoder B Count:", countB)
        time.sleep(1)
except KeyboardInterrupt:
    pass
