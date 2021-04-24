<template>
  <v-card color="base">
    <v-card-title sm="12" class="trajectories-title d-flex justify-center">
      <h3 class="white--text">{{ $t('trajectories.trajectories') }}</h3>
    </v-card-title>
    <v-container>
      <v-row>
        <v-spacer>
          <div
            class="path"
            v-bind:style="{
              backgroundSize: `${this.rescaledWidth}px ${this.rescaledHeight}px`,
              backgroundRepeat: 'no-repeat',
              backgroundImage: `url('${this.tableImage.current}'), url('${this.tableImage.previous}')`,
            }"
          >
            <svg
              :height="this.rescaledHeight"
              :width="this.rescaledWidth"
              id="svg"
            >
              <polyline
                id="planned_path"
                :points="this.plannedTrajectoryPoints"
                style="fill: none; stroke: blue; stroke-width: 2"
              />
              <polyline
                id="real_path"
                :points="this.realTrajectoryPoints"
                style="fill: none; stroke: red; stroke-width: 2"
              />
              <font-awesome-icon
                :icon="['fas', 'map-marker-alt']"
                :height="40"
                :width="40"
                :x="this.startPointX"
                :y="this.startPointY"
                :viewBox="`200 ${this.rescaledWidth} ${this.rescaledWidth} ${this.rescaledHeight}`"
                style="color: blue"
              />
              <font-awesome-icon
                :icon="['fas', 'map-marker-alt']"
                :height="40"
                :width="40"
                :x="this.destinationPointX"
                :y="this.destinationPointY"
                :viewBox="`200 ${this.rescaledWidth} ${this.rescaledWidth} ${this.rescaledHeight}`"
                style="color: red"
              />
            </svg>
          </div>
        </v-spacer>
        <v-col sm="4">
          <Legend />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { Coordinate } from '@/types/coordinate';
import { mapState } from 'vuex';
import Legend from './Legend.vue';
import { TableImage } from '@/types/tableImage';

@Component({
  components: {
    Legend: Legend,
  },
  computed: mapState([
    'tableImage',
    'plannedTrajectory',
    'currentPlannedTrajectory',
    'realTrajectory',
  ]),
})
export default class Trajectories extends Vue {
  private tableImage!: TableImage;
  private plannedTrajectory!: Array<Coordinate>;
  private currentPlannedTrajectory!: Array<Coordinate>;
  private realTrajectory!: Array<Coordinate>;
  private readonly width = 1600;
  private readonly height = 904;
  private readonly ratioX = 0.45;
  private readonly ratioY = 0.45;

  private get rescaledWidth() {
    return this.width * this.ratioX;
  }

  private get rescaledHeight() {
    return this.height * this.ratioY;
  }

  private get plannedTrajectoryPoints() {
    return this.coordinatesToString(this.plannedTrajectory);
  }

  private get realTrajectoryPoints() {
    return this.coordinatesToString(this.realTrajectory);
  }

  private get startPointX() {
    return this.currentPlannedTrajectory[0]
      ? this.currentPlannedTrajectory[0].x * this.ratioX
      : 0;
  }

  private get startPointY() {
    return this.currentPlannedTrajectory[0]
      ? this.currentPlannedTrajectory[0].y * this.ratioY
      : 0;
  }

  private get destinationPointX() {
    return this.currentPlannedTrajectory[
      this.currentPlannedTrajectory.length - 1
    ]
      ? this.currentPlannedTrajectory[this.currentPlannedTrajectory.length - 1]
          .x * this.ratioX
      : 0;
  }

  private get destinationPointY() {
    return this.currentPlannedTrajectory[
      this.currentPlannedTrajectory.length - 1
    ]
      ? this.currentPlannedTrajectory[this.currentPlannedTrajectory.length - 1]
          .y * this.ratioY
      : 0;
  }

  private coordinatesToString(trajectory: Array<Coordinate>) {
    let points = '';
    trajectory.forEach(
      (coordinate) =>
        (points += `${coordinate.x * this.ratioX},${
          coordinate.y * this.ratioY
        } `)
    );
    return points;
  }
}
</script>

<style>
.trajectories-title {
  padding: 0 !important;
  background: #1e1e1e;
}
.trajectories-title > h3 {
  font-size: 1.1rem;
}
</style>
