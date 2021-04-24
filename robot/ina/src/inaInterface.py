from ina219 import INA219

class inaInterface:

    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS_MOTOR = 0.3
    MAX_EXPECTED_AMPS_MAIN = 1
    TOTAL_BATTERY_CHARGE = 8000                 #mA
    TOTAL_BATTERY_CHARGE_LOST_PER_VOLT = 2000   #mA
    FULLY_CHARGE_VOLTAGE = 12
    PENTE_DECHARGE = 1.698468
    CHARGE_MINIMALE = 3

    def __init__(self, readFrequencyPerMesureTime, mesureTime):

        self.numberOfRead = readFrequencyPerMesureTime
        self.interval = mesureTime/readFrequencyPerMesureTime
        self.mesureTime = mesureTime
        self.inaMain = INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS_MAIN, address=0x40)
        self.inaM1 = INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS_MOTOR, address=0x41)
        self.inaM2 = INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS_MOTOR, address=0x42)
        self.inaM3 = INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS_MOTOR, address=0x43)
        self.inaM4 = INA219(self.SHUNT_OHMS, self.MAX_EXPECTED_AMPS_MOTOR, address=0x44)

        self.inaMain.configure(self.inaMain.RANGE_16V, self.inaMain.GAIN_AUTO)
        self.inaM1.configure(self.inaM1.RANGE_16V, self.inaM1.GAIN_AUTO)
        self.inaM2.configure(self.inaM2.RANGE_16V, self.inaM2.GAIN_AUTO)
        self.inaM3.configure(self.inaM3.RANGE_16V, self.inaM3.GAIN_AUTO)
        self.inaM4.configure(self.inaM4.RANGE_16V, self.inaM4.GAIN_AUTO)

        self.index = 0

        self.powerMain = 0
        self.voltageMain = 0
        self.currentMain = 0
        self.powerM1 = 0
        self.powerM2 = 0
        self.powerM3 = 0
        self.powerM4 = 0

        self.powerCalculatorMain = 0
        self.voltageCalculatorMain = 0
        self.currentCalculatorMain = 0
        self.currentCalculatorM1 = 0
        self.currentCalculatorM2 = 0
        self.currentCalculatorM3 = 0
        self.currentCalculatorM4 = 0

        #Calcul de la charge restante.  (Ah)


        self.startCharge = self.estimateChargeRemaining(self.mesureInitVoltage())
        self.secondElapsed = 0

        self.remainingTime = 0
        self.totalCurrentUsed = 0

        self.m1Voltage = 0
        self.m2Voltage = 0
        self.m3Voltage = 0
        self.m4Voltage = 0


        #méthode à appeler à chaque interval de temps.
    def makePowerMesurement(self):
        try:
            self.powerCalculatorMain += self.inaMain.power()
            self.voltageCalculatorMain += self.inaMain.voltage()
            self.currentCalculatorMain += self.inaMain.current()
            self.currentCalculatorM1 += self.inaM1.current()
            self.currentCalculatorM2 += self.inaM2.current()
            self.currentCalculatorM3 += self.inaM3.current()
            self.currentCalculatorM4 += self.inaM4.current()
            self.index += 1
        except Exception:
            pass

        if self.index >= self.numberOfRead:
            self.powerMain = self.powerCalculatorMain/self.numberOfRead
            self.voltageMain = self.voltageCalculatorMain/self.numberOfRead
            self.currentMain =  self.currentCalculatorMain/self.numberOfRead
            self.powerM1 = abs(self.m1Voltage * (self.currentCalculatorM1/self.numberOfRead))
            self.powerM2 = abs(self.m2Voltage * (self.currentCalculatorM2/self.numberOfRead))
            self.powerM3 = abs(self.m3Voltage * (self.currentCalculatorM3/self.numberOfRead))
            self.powerM4 = abs(self.m4Voltage * (self.currentCalculatorM4/self.numberOfRead))

            self.powerCalculatorMain = 0
            self.voltageCalculatorMain = 0
            self.currentCalculatorMain = 0
            self.currentCalculatorM1 = 0
            self.currentCalculatorM2 = 0
            self.currentCalculatorM3 = 0
            self.currentCalculatorM4 = 0
            self.index = 0


            self.secondElapsed += self.mesureTime

            self.totalCurrentUsed += self.currentMain * (1 / 1000) * (self.mesureTime / 3600)    #mAh
            self.remainingTime = 0


    def getInterval(self):
        return self.interval

    def getNumberOfRead(self):
        return self.numberOfRead


    def getMesuredPower(self):
        return self.powerMain



    def getRemainingSecond(self):
        remainingCharge = self.startCharge - self.totalCurrentUsed
        if remainingCharge <= self.CHARGE_MINIMALE:
            return 0
        penteCurrentUsed = self.totalCurrentUsed / max(self.secondElapsed, 1)
        return round(remainingCharge/max(penteCurrentUsed, 0.0001))



    def updateMotorVoltage(self, motor, mValue):
        if motor == 1:
            self.m1Voltage = (mValue/255) * 12

        if motor == 2:
            self.m2Voltage = (mValue/255) * 12

        if motor == 3:
            self.m3Voltage = (mValue/255) * 12

        if motor == 4:
            self.m4Voltage = (mValue/255) * 12


    def printResults(self):
        print("============================================")
        print("Main Power: " + str(round(self.powerMain, 2)) + " mW")
        print("Main Current: " + str(round(self.currentMain, 2)) + "mA")
        print("============================================")
        print("M1 Power: " + str(round(self.powerM1, 2)) + "mW")
        print("M2 Power: " + str(round(self.powerM2, 2)) + "mW")
        print("M3 Power: " + str(round(self.powerM3, 2)) + "mW")
        print("M4 Power: " + str(round(self.powerM4, 2)) + "mW")
        print("============================================")




    def mesureInitVoltage(self):
        voltageCalculator = 0
        nbOfMesure = 1000
        for _ in range(0, nbOfMesure):
            voltageCalculator += self.inaMain.voltage()

        return voltageCalculator / nbOfMesure




    def estimateChargeRemaining(self, voltage):

        lostVoltage = 12.27 - voltage
        remainingCharge = 8 - lostVoltage * self.PENTE_DECHARGE
        return remainingCharge
