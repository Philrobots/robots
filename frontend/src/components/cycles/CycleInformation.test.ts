import CycleInformation from '@/components/cycles/CycleInformation.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';

const wrapper = wrapComponentForTest(CycleInformation);

describe('When mounting cycle information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
