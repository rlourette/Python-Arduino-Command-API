#!/usr/bin/env python
from Arduino import Arduino
import time

def test(pin, baud, port=""):
    board = Arduino(baud, port=port, timeout=5)
    # board.Melody(pin, ["C4", "G3", "G3", "A3", "G3", 0, "B3", "C4"],
    #              [4, 8, 8, 4, 4, 4, 4, 4])
    # Darth Vader theme(Imperial March) - Star wars
    board.Melody(pin, [
        "A4", "A4", "A4", "A4", "A4", "A4", "F4", "REST",
        "A4", "A4", "A4", "A4", "A4", "A4", "F4", "REST",
        "A4", "A4", "A4", "F4", "C5",

        "A4", "F4", "C5", "A4", # 4
        "E5", "E5", "E5", "F5", "C5",
        "A4", "F4", "C5", "A4",

        "A5", "A4", "A4", "A5", "GS5", "G5", # 7
        "DS5", "D5", "DS5", "REST", "A4", "DS5", "D5", "CS5",

        "C5", "B4", "C5", "REST", "F4", "GS4", "F4", "A4", # 9
        "C5", "A4", "C5", "E5",

        "A5", "A4", "A4", "A5", "GS5", "G5", # 7
        "DS5", "D5", "DS5", "REST", "A4", "DS5", "D5", "CS5",

        "C5", "B4", "C5", "REST", "F4", "GS4", "F4", "A4", # 9
        "A4", "F4", "C5", "A4",
        ], [
        -4, -4, 16, 16, 16, 16, 8, 8,
        -4, -4, 16, 16, 16, 16, 8, 8,
        4, 4, 4, -8, 16,

        4, -8, 16, 2, # 4
        4, 4, 4, -8, 16,
        4, -8, 16, 2,

        4, -8, 16, 4, -8, 16, # 7
        16, 16, 8, 8, 8, 4, -8, 16,

        16, 16, 16, 8, 8, 4, -8, -16, # 9
        4, -8, 16, 2,

        4, -8, 16, 4, -8, 16, # 7
        16, 16, 8, 8, 8, 4, -8, 16,

        16, 16, 16, 8, 8, 4, -8, -16, # 9
        4, -8, 16, 2])

def Blink(led_pin, baud, port=""):
    """
    Blinks an LED in 1 sec intervals
    """
    board = Arduino(baud, port=port, timeout=5)
    # time.sleep(1)
    board.pinMode(led_pin, "OUTPUT")
    while True:
        board.digitalWrite(led_pin, "LOW")
        print(board.digitalRead(led_pin))  # confirm LOW (0)
        time.sleep(1)
        board.digitalWrite(led_pin, "HIGH")
        print(board.digitalRead(led_pin))  # confirm HIGH (1)
        time.sleep(1)


def softBlink(led_pin, baud, port=""):
    """
    Fades an LED off and on, using
    Arduino's analogWrite (PWM) function
    """
    board = Arduino(baud, port=port)
    board.pinMode(led_pin, "OUTPUT")

    step = 4
    while True:
        for k in range(0,256,step):
            board.analogWrite(led_pin, k)
        for k in range(k-step, -1, -step):
            board.analogWrite(led_pin, k)

def adjustBrightness(pot_pin, led_pin, baud, port=""):
    """
    Adjusts brightness of an LED using a
    potentiometer.
    """
    board = Arduino(baud, port=port)
    while True:
        time.sleep(0.01)
        val = board.analogRead(pot_pin) / 4
        print(val)
        board.analogWrite(led_pin, val)


def PingSonar(pw_pin, baud, port=""):
    """
    Gets distance measurement from Ping)))
    ultrasonic rangefinder connected to pw_pin
    """
    board = Arduino(baud, port=port)
    pingPin = pw_pin
    while True:
        duration = board.pulseIn(pingPin, "HIGH")
        inches = duration / 72. / 2.
        # cent = duration / 29. / 2.
        print(inches, "inches")
        time.sleep(0.1)


def LCD(tx, baud, ssbaud, message, port=""):
    """
    Prints to two-line LCD connected to
    pin tx
    """
    board = Arduino(baud, port=port)
    board.SoftwareSerial.begin(0, tx, ssbaud)
    while True:
        board.SoftwareSerial.write(" test ")

if __name__ == "__main__":
   #Blink(4, 9600, port="com3")
   test(5, 9600, port="com3")
   # softBlink(6, 9600, port="com3")

def float2intFraction(val, accuracy):
   '''
   Routine to convert decimal values to integer fractions.
   Written by:     Erik Oosterwal
   Started on:     November 9, 2005
   Completed on:   November 9, 2005
   '''

   # Dim intNumerator, intDenominator, intNegative As Long   ' Declare integer variables as long integers.
   # Dim dblFraction, dblDecimal, dblAccuracy As Double      ' Declare floating point variables as double precision.

   dblDecimal = val  # Get the desired decimal value from the front panel (VB Form).
   dblAccuracy = accuracy  # Get the desired accuracy from the front
   # '   panel (VB Form).  Need to put a check
   # '   in here to make sure the desired
   # '   accuracy is not smaller than what the
   # '   programming language can handle.  For
   # '   VB, this should be 1E-13.

   intNumerator = 0  # ' Set the initial numerator value to 0.
   intDenominator = 1  # ' Set the initial denominator value to 1.
   intNegative = 1  # ' Set the negative value flag to positive.

   if dblDecimal < 0:
       intNegative = -1  # ' If the desired decimal value is negative,
       # '   then set the negative value flag to
       # '   negative.

   dblFraction = 0  # ' Set the fraction value to be 0/1.

   while abs(dblFraction - dblDecimal) > dblAccuracy:  # ' As long as we're still outside the
       # '   desired accuracy, then...
       if abs(dblFraction) > abs(dblDecimal):  # ' If our fraction is too big,
           intDenominator = intDenominator + 1  # '   increase the denominator
       else:  # ' Otherwise
           intNumerator = intNumerator + intNegative  # '   increase the numerator.

       dblFraction = intNumerator / intDenominator  # ' Set the new value of the fraction.

   print(f"Numerator({intNumerator}), Denominator({intDenominator})")  # ' Display the numerator and denominator
