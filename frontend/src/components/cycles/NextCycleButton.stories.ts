import NextCycleButton from '@/components/cycles/NextCycleButton.vue';
import Vuex from 'vuex';
import { Step } from '@/types/step';

export default {
  title: 'components/cycles/NextCycleButton',
  component: NextCycleButton,
};

const Template = (args: any) => ({
  components: { NextCycleButton },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<NextCycleButton  v-bind="$props" />',
});

export const CycleNotEnded = Template.bind({}) as any;
CycleNotEnded.args = {
  currentStep: Step.CycleStarted,
};

export const CycleEnded = Template.bind({}) as any;
CycleEnded.args = {
  currentStep: Step.CycleEndedAndRedLedOn,
};
