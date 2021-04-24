import Chronometer from '@/components/cycles/Chronometer.vue';
import { Step } from '@/types/step';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/Chronometer',
  component: Chronometer,
};

const Template = (args: any) => ({
  components: { Chronometer },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Chronometer />',
});

export const WhenCycleNotStartedAndCycleReady = Template.bind({}) as any;
WhenCycleNotStartedAndCycleReady.args = {
  currentStep: Step.CycleNotStarted,
  cycleStarted: false,
  cycleReady: true,
};

export const WhenCycleNotStartedAndCycleNotReady = Template.bind({}) as any;
WhenCycleNotStartedAndCycleNotReady.args = {
  currentStep: Step.CycleNotStarted,
  cycleStarted: false,
  cycleReady: false,
};

export const WhenCycleEnds = Template.bind({}) as any;
WhenCycleEnds.args = {
  currentStep: Step.CycleEndedAndRedLedOn,
  cycleStarted: true,
  cycleReady: true,
};
