import StartButton from '@/components/cycles/StartButton.vue';
import Vuex from 'vuex';

export default {
  title: 'components/cycles/StartButton',
  component: StartButton,
};

const Template = (args: any) => ({
  components: { StartButton },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<StartButton  v-bind="$props" />',
});

export const CycleNotReady = Template.bind({}) as any;
CycleNotReady.args = {
  cycleReady: false,
  cycleStarted: false,
};

export const CycleReady = Template.bind({}) as any;
CycleReady.args = {
  cycleReady: true,
  cycleStarted: false,
};

export const CycleStarted = Template.bind({}) as any;
CycleStarted.args = {
  cycleReady: true,
  cycleStarted: true,
};
