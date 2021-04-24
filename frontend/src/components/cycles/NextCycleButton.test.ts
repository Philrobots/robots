import wrapComponentForTest from '../../util/wrapComponentForTest';
import NextCycleButton from '@/components/cycles/NextCycleButton.vue';

describe('When mounting NextCycle component', () => {
  const wrapper = wrapComponentForTest(NextCycleButton);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
