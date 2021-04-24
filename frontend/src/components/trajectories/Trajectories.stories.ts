import Trajectories from '@/components/trajectories/Trajectories.vue';
import Vue from 'vue';
import Vuex from 'vuex';
import { CoordinateFactory } from '@/factories/CoordinateFactory';
import { TableImage } from '@/types/tableImage';

Vue.use(Vuex);

export default {
  title: 'components/trajectories/Trajectories',
  component: Trajectories,
};

const Template = (args: any) => ({
  components: { Trajectories },
  store: new Vuex.Store({
    state: args,
  }),
  template: '<trajectories/>',
});

const plannedTrajectory = CoordinateFactory.make(4);
const simplifyTableImage = (tableImage: string): TableImage => ({
  current: tableImage,
  previous: tableImage,
});

export const WithCurrentTrajectorySameAsPlanned = Template.bind({}) as any;
WithCurrentTrajectorySameAsPlanned.args = {
  tableImage: simplifyTableImage('/stub_table_image.jpg'),
  plannedTrajectory,
  currentPlannedTrajectory: plannedTrajectory,
  realTrajectory: [],
};

export const WithCurrentTrajectoryPartOfPlanned = Template.bind({}) as any;
WithCurrentTrajectoryPartOfPlanned.args = {
  tableImage: simplifyTableImage('/stub_table_image.jpg'),
  plannedTrajectory,
  currentPlannedTrajectory: plannedTrajectory.slice(0, 2),
  realTrajectory: [],
};

export const withoutTrajectories = Template.bind({}) as any;
withoutTrajectories.args = {
  tableImage: simplifyTableImage('/stub_table_image.jpg'),
  plannedTrajectory: [],
  currentPlannedTrajectory: [],
  realTrajectory: [],
};

const plannedTrajectoryToYellowPuck = [
  { x: 1037.5, y: 512.5 },
  { x: 1037.5, y: 537.5 },
  { x: 1012.5, y: 537.5 },
  { x: 1012.5, y: 562.5 },
  { x: 987.5, y: 562.5 },
  { x: 987.5, y: 587.5 },
  { x: 962.5, y: 587.5 },
  { x: 962.5, y: 612.5 },
  { x: 937.5, y: 612.5 },
  { x: 912.5, y: 612.5 },
  { x: 887.5, y: 612.5 },
  { x: 862.5, y: 612.5 },
  { x: 837.5, y: 612.5 },
  { x: 812.5, y: 612.5 },
  { x: 787.5, y: 612.5 },
  { x: 787.5, y: 587.5 },
  { x: 762.5, y: 587.5 },
  { x: 762.5, y: 562.5 },
  { x: 737.5, y: 562.5 },
  { x: 737.5, y: 537.5 },
  { x: 712.5, y: 537.5 },
  { x: 712.5, y: 512.5 },
  { x: 687.5, y: 512.5 },
  { x: 687.5, y: 487.5 },
  { x: 662.5, y: 487.5 },
  { x: 662.5, y: 462.5 },
  { x: 637.5, y: 462.5 },
  { x: 637.5, y: 437.5 },
  { x: 637.5, y: 412.5 },
  { x: 612.5, y: 412.5 },
  { x: 587.5, y: 412.5 },
  { x: 562.5, y: 412.5 },
  { x: 537.5, y: 412.5 },
];

export const TrajectoryToYellowPuck = Template.bind({}) as any;
TrajectoryToYellowPuck.args = {
  tableImage: simplifyTableImage('/stub_table_image1.jpg'),
  plannedTrajectory: plannedTrajectoryToYellowPuck,
  currentPlannedTrajectory: plannedTrajectoryToYellowPuck,
  realTrajectory: [],
};

const realTrajectoryToYellowPuck = [
  { x: 1037.5, y: 512.5 },
  { x: 1037.5, y: 537.5 },
  { x: 1012.5, y: 537.5 },
  { x: 1012.5, y: 562.5 },
  { x: 987.5, y: 562.5 },
  { x: 987.5, y: 587.5 },
  { x: 962.5, y: 587.5 },
];

export const RealTrajectoryToYellowPuck = Template.bind({}) as any;
RealTrajectoryToYellowPuck.args = {
  tableImage: simplifyTableImage('/stub_table_image1.jpg'),
  plannedTrajectory: plannedTrajectoryToYellowPuck,
  currentPlannedTrajectory: plannedTrajectoryToYellowPuck,
  realTrajectory: realTrajectoryToYellowPuck,
};
