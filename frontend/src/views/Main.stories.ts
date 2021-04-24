import Main from '@/views/Main.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

Vue.use(Vuex);

export default {
  title: 'views/Main',
  component: Main,
};

export const Default = () => ({
  components: { Main },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<Main />`,
});
