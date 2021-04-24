import StepList from '@/components/cycles/StepList.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { Step } from '@/types/step';
import { State } from '@/store/state';

describe('When mounting StepList component', () => {
  const wrapper = wrapComponentForTest(StepList);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    currentStep: Step.CycleNotStarted,
  } as State;

  describe('When mounting StepList', () => {
    const wrapper = wrapComponentForTest(StepList, state);

    it('Should contains the right amount of steps', () => {
      const steps = wrapper.findAllComponents({ ref: 'step' });

      expect(steps.exists()).toBe(true);
      expect(steps).toHaveLength(15);
    });
  });
});
