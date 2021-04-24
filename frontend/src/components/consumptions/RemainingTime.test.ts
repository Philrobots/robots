import RemainingTime from '@/components/consumptions/RemainingTime.vue';
import { State } from '@/store/state';
import wrapComponentForTest from '../../util/wrapComponentForTest';

describe('Given state', () => {
  const state = {
    robotConsumption: {
      wheel1: 0,
      wheel2: 0,
      wheel3: 0,
      wheel4: 0,
      total: 0,
      remainingTime: 60,
      batteryCharge: 0,
    },
  } as State;

  describe('When mounting RemainingTime', () => {
    const wrapper = wrapComponentForTest(RemainingTime, state);

    it('Should contains the right time in hh:mm:ss', () => {
      const time = wrapper.findComponent({ ref: 'time' });

      expect(time.exists()).toBe(true);
      expect(time.text()).toBe('00:01:00');
    });
  });
});
