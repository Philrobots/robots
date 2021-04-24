import FirstCorner from '@/components/objectives/FirstCorner.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { Corner } from '@/types/corner';
import { PuckListFactory } from '@/factories/PuckListFactory';

Vue.use(Vuex);

export default {
  title: 'components/objectives/FirstCorner',
  component: FirstCorner,
};

const mockPuckList = (firstCorner: Corner) => {
  const puckList = PuckListFactory.make();
  puckList.firstCorner = firstCorner;
  return puckList;
};

const Template = (args: any) => ({
  components: { ControlPanel: FirstCorner },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<ControlPanel/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckList: PuckListFactory.make(),
};

export const cornerA = Template.bind({}) as any;
cornerA.args = {
  puckList: mockPuckList(Corner.A),
};

export const cornerB = Template.bind({}) as any;
cornerB.args = {
  puckList: mockPuckList(Corner.B),
};

export const cornerC = Template.bind({}) as any;
cornerC.args = {
  puckList: mockPuckList(Corner.C),
};

export const cornerD = Template.bind({}) as any;
cornerD.args = {
  puckList: mockPuckList(Corner.D),
};
