import Mode from '@/components/cycles/Mode.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/cycles/Mode',
  component: Mode,
};

const Template = (args: any) => ({
  components: { Mode },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<Mode />',
});

export const CycleReadyInWaitingMode = Template.bind({}) as any;
CycleReadyInWaitingMode.args = {
  cycleReady: true,
  cycleStarted: false,
};

export const CycleStarted = Template.bind({}) as any;
CycleStarted.args = {
  cycleReady: true,
  cycleStarted: true,
};

export const CycleNotStartedAndRobotStillBooting = Template.bind({}) as any;
CycleNotStartedAndRobotStillBooting.args = {
  cycleReady: false,
  cycleStarted: false,
};
