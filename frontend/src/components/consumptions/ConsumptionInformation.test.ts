import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';

const wrapper = wrapComponentForTest(ConsumptionInformation);

describe('When mounting consumption information', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});
