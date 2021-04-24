import ObjectivesInformation from '@/components/objectives/ObjectivesInformation.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

Vue.use(Vuex);

export default {
  title: 'components/objectives/ObjectivesInformation',
  component: ObjectivesInformation,
};

export const Default = () => ({
  components: { StationInformation: ObjectivesInformation },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<station-information/>`,
});
