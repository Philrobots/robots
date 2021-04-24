import RobotConsumptionInfo from '@/components/consumptions/RobotConsumptionInfo.vue';
import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default {
  title: 'components/consumptions/RobotConsumptionInfo',
  component: RobotConsumptionInfo,
};

const Template = (args: any) => ({
  components: { RobotConsumptionInfo },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<RobotConsumptionInfo />',
});

export const anyData = Template.bind({}) as any;
anyData.args = {
  robotConsumption: {
    wheel1: 80.9,
    wheel2: 4,
    wheel3: 0.01,
    wheel4: 0.2,
    servoMotor: 10,
    total: 0,
  },
};
