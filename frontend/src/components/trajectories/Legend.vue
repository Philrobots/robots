<template>
  <v-card color="lighten2" elevation="5">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('trajectories.legend') }}</h5>
    </v-card-title>
    <v-container>
      <v-row>
        <svg height="120" width="400" id="legend">
          <text :x="xInitialPlacement" :y="yInitialPlacement" fill="white">
            {{ `${$t('trajectories.plannedTrajectory')} :` }}
          </text>
          <line
            :x1="x1StrokeInitialPlacement"
            :y1="yStrokeInitialPlacement"
            :x2="x2StrokeInitialPlacement"
            :y2="yStrokeInitialPlacement"
            style="stroke: blue; stroke-width: 2"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaRealTrajectory"
            fill="white"
          >
            {{ `${$t('trajectories.realTrajectory')} :` }}
          </text>
          <line
            :x1="x1StrokeInitialPlacement"
            :y1="yStrokeInitialPlacement + deltaRealTrajectory"
            :x2="x2StrokeInitialPlacement"
            :y2="yStrokeInitialPlacement + deltaRealTrajectory"
            style="stroke: red; stroke-width: 2"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaStartingPoint"
            fill="white"
          >
            {{ `${$t('trajectories.startingPoint')} :` }}
          </text>
          <font-awesome-icon
            :icon="['fas', 'map-marker-alt']"
            :height="20"
            :width="20"
            :x="x1StrokeInitialPlacement - 11"
            :y="yInitialPlacement + deltaStartingPoint - 20"
            style="color: blue"
          />
          <text
            :x="xInitialPlacement"
            :y="yInitialPlacement + deltaEndingPoint"
            fill="white"
          >
            {{ `${$t('trajectories.destinationPoint')} :` }}
          </text>
          <font-awesome-icon
            :icon="['fas', 'map-marker-alt']"
            :height="20"
            :width="20"
            :x="x1StrokeInitialPlacement - 11"
            :y="yInitialPlacement + deltaEndingPoint - 15"
            style="color: red"
          />
        </svg>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({})
export default class Legend extends Vue {
  public xInitialPlacement = 10;
  public yInitialPlacement = 20;

  public x1StrokeInitialPlacement = 180;
  public x2StrokeInitialPlacement = 250;
  public yStrokeInitialPlacement = 15;

  public deltaRealTrajectory = 30;
  public deltaStartingPoint = 60;
  public deltaEndingPoint = 90;
}
</script>

<style>
.v-card__title {
  padding: 0 !important;
}
</style>
