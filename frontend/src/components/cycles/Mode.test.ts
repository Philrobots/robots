import Mode from '@/components/cycles/Mode.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';

const mockState = (cycleReady: boolean, cycleStarted: boolean) =>
  ({
    cycleReady: cycleReady,
    cycleStarted: cycleStarted,
  } as State);

describe('When mounting Mode component', () => {
  const wrapper = wrapComponentForTest(Mode);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given booting state', () => {
  const state = mockState(false, false);

  describe('When mounting Mode', () => {
    const wrapper = wrapComponentForTest(Mode, state);

    it('Should be booting mode', () => {
      const mode = wrapper.findComponent({ ref: 'mode' });

      expect(mode.exists()).toBe(true);
      expect(mode.text()).toBe(wrapper.vm.$t('cycles.modes.booting'));
    });
  });
});

describe('Given waiting state', () => {
  const state = mockState(true, false);

  describe('When mounting Mode', () => {
    const wrapper = wrapComponentForTest(Mode, state);

    it('Should be booting mode', () => {
      const mode = wrapper.findComponent({ ref: 'mode' });

      expect(mode.exists()).toBe(true);
      expect(mode.text()).toBe(wrapper.vm.$t('cycles.modes.waiting'));
    });
  });
});

describe('Given started state', () => {
  const state = mockState(true, true);

  describe('When mounting Mode', () => {
    const wrapper = wrapComponentForTest(Mode, state);

    it('Should be started mode', () => {
      const mode = wrapper.findComponent({ ref: 'mode' });

      expect(mode.exists()).toBe(true);
      expect(mode.text()).toBe(wrapper.vm.$t('cycles.modes.started'));
    });
  });
});
