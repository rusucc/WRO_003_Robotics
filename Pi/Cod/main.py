import numpy as np
from time import monotonic
import cv2
import serial
import src.comanda_directie
import src.FiniteStateMachine as FSM

"""ACESTA ESTE CODUL DE MAIN PENTRU PROBA FARA OBSTACOLE"""
#init serial#
port_serial = '/dev/tty/USB0'
ser_pico = serial.Serial(port_serial, baudrate= 115200)
ser_pico.print("stiu_ca_vrei_si_stiu_ca_poti")


masina = FSM.FSM("STARE_INITIALA")
print(masina)

if(masina.get_state() == "STARE_INITIALA"):
    ser_pico.print("stiu_ca_vrei_si_stiu_ca_poti")
    try:
        ser_pico.read(timeout=1)
        masina.transition("STARE_LAP")
    except:
        print("nu am primit date")
        masina.transition("STARE_FAULT")
    
if(masina.get_state() == "STARE_LAP"):
    print(masina)
    ser_pico.print()
if(masina.get_state() == "STARE_FAULT"):
    print(masina)
    
if(masina.get_state() == "STARE_FINALA"):
