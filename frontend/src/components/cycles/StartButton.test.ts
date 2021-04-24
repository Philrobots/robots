import wrapComponentForTest from '../../util/wrapComponentForTest';
import StartButton from '@/components/cycles/StartButton.vue';

describe('When mounting StartButton component', () => {
  const wrapper = wrapComponentForTest(StartButton);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
