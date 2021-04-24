import RobotConsumptionInfo from '@/components/consumptions/RobotConsumptionInfo.vue';
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
      batteryCharge: 0,
    } as RobotConsumption,
  } as State;

  describe('When mounting RobotConsumptionInfo', () => {
    const wrapper = wrapComponentForTest(RobotConsumptionInfo, state);

    it('Should contains the right number of consumptions details', () => {
      const consumptions = wrapper.findComponent({ ref: 'consumptions' });
      const everyConsumption = consumptions.findAll('h5');

      expect(everyConsumption.exists()).toBe(true);
      expect(everyConsumption).toHaveLength(5);
    });
  });
});
