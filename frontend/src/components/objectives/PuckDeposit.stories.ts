import PuckDeposit from '@/components/objectives/PuckDeposit.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { PuckState } from '@/types/puckState';

Vue.use(Vuex);

export default {
  title: 'components/objectives/PuckDeposit',
  component: PuckDeposit,
};

const Template = (args: any) => ({
  components: { PuckDeposit },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<PuckDeposit v-bind="$props"/>',
});

export const Default = Template.bind({}) as any;
Default.args = {
  puckList: PuckListFactory.make(),
};

export const NothingDeposited = Template.bind({}) as any;
NothingDeposited.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]),
};

export const FirstPuckGripped = Template.bind({}) as any;
FirstPuckGripped.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]),
};

export const FirstPuckReleased = Template.bind({}) as any;
FirstPuckReleased.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]),
};

export const SecondPuckGripped = Template.bind({}) as any;
SecondPuckGripped.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.RELEASED,
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
  ]),
};

export const SecondPuckReleased = Template.bind({}) as any;
SecondPuckReleased.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
  ]),
};

export const ThirdPuckGripped = Template.bind({}) as any;
ThirdPuckGripped.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.GRIPPED,
  ]),
};

export const ThirdPuckReleased = Template.bind({}) as any;
ThirdPuckReleased.args = {
  puckList: PuckListFactory.makeWithStates([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.RELEASED,
  ]),
};
