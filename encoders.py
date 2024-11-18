from machine import Pin
from config import ENCODER_A_PIN_A, ENCODER_A_PIN_B, ENCODER_B_PIN_A, ENCODER_B_PIN_B

class Encoders:
    def __init__(self):

# These define the encoder pins for both encoders
        self.encoderA_A = Pin(ENCODER_A_PIN_A, Pin.IN)
        self.encoderA_B = Pin(ENCODER_A_PIN_B, Pin.IN)
        self.encoderB_A = Pin(ENCODER_B_PIN_A, Pin.IN)
        self.encoderB_B = Pin(ENCODER_B_PIN_B, Pin.IN)
        
# These are to keep count of the number of encoder ticks for each encoder
        self.countA = 0
        self.countB = 0
        
# These are the interrupts which is something that takes priority so if the pin goes from low to high, then the handler(response) will be triggered
        self.encoderA_A.irq(trigger=Pin.IRQ_RISING, handler=self.encoderA_callback)
        self.encoderB_A.irq(trigger=Pin.IRQ_RISING, handler=self.encoderB_callback)

# These are the actual responses from the interrupts above
# These take into consideration the way that the encoders work and take advantage of that by seeing if both pins A and B for encoder A are the same (high or low) or different to determine if the wheels are moving forwards or backwards
    def encoderA_callback(self, pin):
        if self.encoderA_A.value() == self.encoderA_B.value():
            self.countA += 1
        else:
            self.countA -= 1

# This is the same as encoder A but just for encoder B     
    def encoderB_callback(self, pin):
        if self.encoderB_A.value() == self.encoderB_B.value():
            self.countB += 1
        else:
            self.countB -= 1

# This a function to set the counts back to zero that will be useful when beginning a new measurement     
    def reset_counts(self):
        self.countA = 0
        self.countB = 0

# This function returns the encoder counts but as values without the direction so without positive or negative (absolute value) so we know how far the micromouse has moved either or forwards or backwards that we choose
    def get_counts(self):
        return abs(self.countA), abs(self.countB)
