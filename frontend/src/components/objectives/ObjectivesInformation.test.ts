import ObjectivesInformation from '@/components/objectives/ObjectivesInformation.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';

const wrapper = wrapComponentForTest(ObjectivesInformation);

describe('When mounting objectives information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
