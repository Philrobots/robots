import Vue from 'vue';
import Vuex from 'vuex';
import CycleNumber from '@/components/cycles/CycleNumber.vue';

Vue.use(Vuex);

export default {
  title: 'components/cycles/CycleNumber',
  component: CycleNumber,
};

const Template = (args: any) => ({
  components: { CycleNumber },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<CycleNumber />',
});

export const Basic = Template.bind({}) as any;
Basic.args = {
  cycleNumber: 88,
};
