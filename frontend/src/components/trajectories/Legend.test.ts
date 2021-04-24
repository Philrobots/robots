import Legend from '@/components/trajectories/Legend.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';

describe('When mounting Legend', () => {
  const wrapper = wrapComponentForTest(Legend);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
