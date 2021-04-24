import StepList from '@/components/cycles/StepList.vue';
import { Step } from '@/types/step';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/StepList',
  component: StepList,
};

const Template = (args: any) => ({
  components: { StepList },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<StepList  v-bind="$props" />',
});

export const AtFirstStep = Template.bind({}) as any;
AtFirstStep.args = {
  currentStep: Step.CycleNotStarted,
};

export const AtFurtherStep = Template.bind({}) as any;
AtFurtherStep.args = {
  currentStep: Step.ReadResistance,
};

export const AtLastStep = Template.bind({}) as any;
AtLastStep.args = {
  currentStep: Step.CycleEndedAndRedLedOn,
};
