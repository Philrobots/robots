import GripState from '@/components/objectives/GripState.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { PuckState } from '@/types/puckState';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { PuckList } from '@/types/puckList';

Vue.use(Vuex);

export default {
  title: 'components/objectives/GripState',
  component: GripState,
};

const Template = (args: any) => ({
  components: { GripState },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<GripState />',
});

export const WithPuckInGrip = Template.bind({}) as any;
WithPuckInGrip.args = {
  puckList: PuckListFactory.makeWithStates(
    Array(PuckList.PUCKS_COUNT).fill(PuckState.UNTOUCHED)
  ),
};

export const WithoutPuckInGrip = Template.bind({}) as any;
WithoutPuckInGrip.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]),
};
