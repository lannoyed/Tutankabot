import RPi.GPIO as GPIO
from time import sleep
import spidev

MyARM_ResetPin = 19 # Pin 4 of connector = BCM19 = GPIO[1]

MySPI_FPGA = spidev.SpiDev()                
MySPI_FPGA.open(0,0)                            #composant dans le bus
MySPI_FPGA.max_speed_hz = 500000     #vitesse par défaur

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MyARM_ResetPin, GPIO.OUT)

GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(0.1)


# initialiasation de Clock
ToSPI = [0x80, 0x00, 0x4F, 0x00, 0x00]          #vitesse = 5 177 344 Hz
FromSPI = MySPI_FPGA.xfer2(ToSPI)

#initialisation des registres
ToSPI = [0x81, 0x00, 0x00, 0x00, 0x01]         #SPI_Reg1
FromSPI = MySPI_FPGA.xfer2(ToSPI)           

ToSPI = [0x82, 0x00, 0x00, 0x00, 0x04]         #SPI_Reg2
FromSPI = MySPI_FPGA.xfer2(ToSPI)

ToSPI = [0x83, 0x00, 0x00, 0x00, 0x03]         #SPI_Reg3
FromSPI = MySPI_FPGA.xfer2(ToSPI)           


GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(1)

ToSPI = [0x05, 0x00, 0x00, 0x00, 0x00]         #SPI_Reg4
FromSPI = MySPI_FPGA.xfer2(ToSPI)
print(FromSPI)

ToSPI = [0x06, 0x00, 0x00, 0x00, 0x00]         #SPI_Reg5
FromSPI = MySPI_FPGA.xfer2(ToSPI)
print(FromSPI)


