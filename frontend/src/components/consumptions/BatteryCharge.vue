<template>
  <div>
    <v-card class="lighten2">
      <v-card-title class="lighten1 d-flex justify-center">
        <h5 class="white--text">
          {{ $t(`consumptions.currentBatteryCharge`) }}
        </h5>
      </v-card-title>
      <v-container>
        <v-row align="center">
          <v-col sm="6">
            <v-progress-circular
              :rotate="-90"
              :size="100"
              :width="15"
              :value="this.pourcentageBatteryLeft"
              color="primary"
            >
              <h4 ref="batteryCharge">{{ currentBatteryCharge }} Ah</h4>
            </v-progress-circular>
          </v-col>
          <v-col sm="6">
            <RemainingTime />
          </v-col>
        </v-row>
      </v-container>
    </v-card>
  </div>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import RemainingTime from '../consumptions/RemainingTime.vue';

@Component({
  components: { RemainingTime: RemainingTime },
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class BatteryCharge extends Vue {
  public robotConsumption!: RobotConsumption;
  public maximumCharge = 8;
  public trailingDecimals = 3;

  private get currentBatteryCharge() {
    return this.robotConsumption.batteryCharge.toFixed(this.trailingDecimals);
  }

  private get pourcentageBatteryLeft() {
    return (this.robotConsumption.batteryCharge / this.maximumCharge) * 100;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}
</style>
