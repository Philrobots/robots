import CycleInformation from '@/components/cycles/CycleInformation.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

Vue.use(Vuex);

export default {
  title: 'components/cycles/CycleInformation',
  component: CycleInformation,
};

export const Default = () => ({
  components: { CycleInformation },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<CycleInformation/>`,
});
