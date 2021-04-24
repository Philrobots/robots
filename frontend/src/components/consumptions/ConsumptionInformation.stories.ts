import ConsumptionInformation from '@/components/consumptions/ConsumptionInformation.vue';
import { StateFactory } from '@/factories/StateFactory';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/ConsumptionInformation',
  component: ConsumptionInformation,
};

export const Default = () => ({
  components: { ConsumptionInformation },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<ConsumptionInformation/>`,
});
