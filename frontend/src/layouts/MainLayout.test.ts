import MainLayout from '@/layouts/MainLayout.vue';
import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import CycleInformation from '@/components/cycles/CycleInformation.vue';
import StationInformation from '@/components/objectives/ObjectivesInformation.vue';
import Trajectories from '@/components/trajectories/Trajectories.vue';
import wrapComponentForTest from '../util/wrapComponentForTest';

const wrapper = wrapComponentForTest(MainLayout);

describe('When mounting main layout', () => {
  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });

  it('Should contain consumption information', () => {
    expect(wrapper.findComponent(ConsumptionInformation).vm).toBeTruthy();
  });

  it('Should contain cycle information', () => {
    expect(wrapper.findComponent(CycleInformation).vm).toBeTruthy();
  });

  it('Should contain objectives information', () => {
    expect(wrapper.findComponent(StationInformation).vm).toBeTruthy();
  });

  it('Should contain trajectories', () => {
    expect(wrapper.findComponent(Trajectories).vm).toBeTruthy();
  });
});
