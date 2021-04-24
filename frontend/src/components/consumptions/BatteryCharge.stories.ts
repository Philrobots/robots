import BatteryCharge from '@/components/consumptions/BatteryCharge.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/BatteryCharge',
  component: BatteryCharge,
};

const Template = (args: any) => ({
  components: { BatteryCharge },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<BatteryCharge />',
});

export const AllChargeLeft = Template.bind({}) as any;
AllChargeLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 8,
  },
};

export const HalfChargeLeft = Template.bind({}) as any;
HalfChargeLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 4,
  },
};

export const ThirdChargeLeft = Template.bind({}) as any;
ThirdChargeLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 2,
  },
};

export const NoChargeLeft = Template.bind({}) as any;
NoChargeLeft.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 0,
  },
};

export const ChargeWith3Decimals = Template.bind({}) as any;
ChargeWith3Decimals.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 2.455,
  },
};

export const ChargeWith2Decimals = Template.bind({}) as any;
ChargeWith2Decimals.args = {
  robotConsumption: {
    wheel1: 0,
    wheel2: 0,
    wheel3: 0,
    wheel4: 0,
    total: 0,
    remainingTime: 0,
    batteryCharge: 5.04,
  },
};
