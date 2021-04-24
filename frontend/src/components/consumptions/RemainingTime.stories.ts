import RemainingTime from '@/components/consumptions/RemainingTime.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/RemainingTime',
  component: RemainingTime,
};

const Template = (args: any) => ({
  components: { RemainingTime },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<RemainingTime />',
});

export const HoursLeft = Template.bind({}) as any;
HoursLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 3 * 60 * 60,
    batteryCharge: 0,
  },
};

export const MinutesLeft = Template.bind({}) as any;
MinutesLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 60 * 40,
    batteryCharge: 0,
  },
};

export const SecondsLeft = Template.bind({}) as any;
SecondsLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 34,
    batteryCharge: 0,
  },
};

export const aWeirdNumber = Template.bind({}) as any;
aWeirdNumber.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 1234,
    batteryCharge: 0,
  },
};
