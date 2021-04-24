import Resistance from '@/components/objectives/Resistance.vue';
import Vuex from 'vuex';
import Vue from 'vue';
import { PuckListFactory } from '@/factories/PuckListFactory';

Vue.use(Vuex);

export default {
  title: 'components/objectives/Resistance',
  component: Resistance,
};

const Template = (args: any) => ({
  components: { Resistance },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Resistance />',
});

export const Default = Template.bind({}) as any;
Default.args = {
  resistance: 100000,
  puckList: PuckListFactory.make(),
};
