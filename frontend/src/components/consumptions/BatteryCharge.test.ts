import BatteryCharge from '@/components/consumptions/BatteryCharge.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import { RobotConsumption } from '@/types/robotConsumption';

describe('Given state', () => {
  const state = {
    robotConsumption: {
      wheel1: 0,
      wheel2: 0,
      wheel3: 0,
      wheel4: 0,
      total: 0,
      remainingTime: 0,
      batteryCharge: 4,
    } as RobotConsumption,
  } as State;

  describe('When mounting BatteryCharge', () => {
    const wrapper = wrapComponentForTest(BatteryCharge, state);

    it('Should contains the right letter of corner', () => {
      const batteryCharge = wrapper.findComponent({ ref: 'batteryCharge' });

      expect(batteryCharge.exists()).toBe(true);
      expect(batteryCharge.text()).toBe('4.000 Ah');
    });
  });
});
