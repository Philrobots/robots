import time
from ina219 import INA219

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.2


def read():
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)

    # print("ici")

    # print("INA1 ==============")
    # print("Bus Voltage    : %.3f V" % ina1.voltage())
    # print("Bus Current    : %.3f mA" % ina1.current())
    # print("Supply Voltage : %.3f V" % ina1.supply_voltage())
    # print("Shunt voltage  : %.3f mV" % ina1.shunt_voltage())
    print("Power          : %.3f mW" % ina1.power())
    # print("")

    print("")
    print("")
    print("")

    # ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    # ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)
    # print("ici")

    # print("INA2 ==============")
    # print("Bus Voltage    : %.3f V" % ina2.voltage())
    # print("Bus Current    : %.3f mA" % ina2.current())
    # print("Supply Voltage : %.3f V" % ina2.supply_voltage())
    # print("Shunt voltage  : %.3f mV" % ina2.shunt_voltage())
    # print("Power          : %.3f mW" % ina2.power())


def timedTest(numberOfReads):
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)

    # ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    # ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)
    begin = time.time()

    averagePower1 = 0
    averagePower2 = 0
    averageCurrent1 = 0
    averageVoltage1 = 0
    averageCurrent2 = 0
    averageVoltage2 = 0

    for i in range(0, numberOfReads):
        averageCurrent1 += ina1.current()
        averageCurrent2 += ina1.current()
        averageVoltage1 += ina1.voltage()
        averageVoltage2 += ina1.voltage()
        averagePower1 += ina1.power()
        averagePower2 += ina1.power()

    averageCurrent1 = averageCurrent1 / numberOfReads
    averageCurrent2 = averageCurrent2 / numberOfReads
    averageVoltage1 = averageVoltage1 / numberOfReads
    averageVoltage2 = averageVoltage2 / numberOfReads
    averagePower1 = averagePower1 / numberOfReads
    averagePower2 = averagePower2 / numberOfReads

    end = time.time()

    print("Average Current #1 ======= ")
    print(str(averageCurrent1) + "mA")
    print("")
    print("Average Voltage #1 ======= ")
    print(str(averageVoltage1) + "V")
    print("")
    print("Average Power #1 ======= ")
    print(str(averagePower1) + "mW")
    print("")
    print("Average Current #2 ======= ")
    print(str(averageCurrent2) + "mA")
    print("")
    print("Average Voltage #2 ======= ")
    print(str(averageVoltage2) + "V")
    print("")
    print("Average Power #2 ======= ")
    print(str(averagePower2) + "mW")
    print("")
    print("Time used =======")
    print(end - begin)


def average():
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)

    ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)

    averageCurrent = 0
    averageTension = 0
    averagePower = 0

    begin = time.time()
    for i in range(0, 90):
        averageCurrent += ina1.current()
        averageTension += ina2.voltage()
        averagePower += ina1.power()

    averageCurrent = averageCurrent / 90
    averageTension = averageTension / 90
    averagePower = averagePower / 90

    end = time.time()
    print("")
    print("Average Current ======= ")
    print(str(averageCurrent) + "mA")
    print("Average Tension ======= ")
    print(str(averageTension) + "V")
    print("Average Power ======= ")
    print(str(averagePower) + "mW")
    print("")
    print("Time used =======")
    print(end - begin)
    return 3


if __name__ == "__main__":
    timedTest(100)
