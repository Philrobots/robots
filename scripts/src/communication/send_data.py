import time
import serial

ser = serial.Serial('/dev/ttyACM0',9600, timeout=2)
ser.flush()

action = str(input('Quel action effectué? :'))

if int(action) <= 5:
    value = input('Quelle valeur associé à cet action?:')
    if ser.isOpen():
        print("is open")
    ser.write(str.encode(action + ':' + str(value)))
    print('Commande envoyé: ' + str(action) + ':' + str(value))
else:
    ser.write(str.encode(action + ':'))
    print('Commande envoyé: ' + str(action) + ':')



while True:
    if ser.in_waiting != 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

actionList = [	'0:1',
                '1:1',
                '2:1',
                '3:1',
                '4:1',
                '5:1',
                '6:',
                '7:'
                '8:']

time.sleep(1)


for x in actionList:
    ser.write(x)
    while ser.in_waiting == 0:
        pass
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    time.sleep(5)
