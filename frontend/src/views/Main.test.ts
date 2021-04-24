import Main from '@/views/Main.vue';
import wrapComponentForTest from '../util/wrapComponentForTest';

const wrapper = wrapComponentForTest(Main);

describe('When mounting main view', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
