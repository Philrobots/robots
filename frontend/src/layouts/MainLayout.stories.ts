import MainLayout from '@/layouts/MainLayout.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { StateFactory } from '@/factories/StateFactory';

Vue.use(Vuex);

export default {
  title: 'layouts/MainLayout',
  component: MainLayout,
};

export const Default = () => ({
  components: { MainLayout },
  store: new Vuex.Store({
    state: StateFactory.make(),
  }),
  template: `<main-layout/>`,
});
