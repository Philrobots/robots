<template>
  <v-card class="lighten2" height="100%">
    <v-card-title class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t(`consumptions.robotConsumption`) }}</h5>
    </v-card-title>
    <v-container ref="consumptions">
      <v-row align="center">
        <v-col sm="6">
          <h5>{{ $t(`consumptions.wheel1`) }} : {{ wheel1Info }}</h5>
          <h5>{{ $t(`consumptions.wheel2`) }} : {{ wheel2Info }}</h5>
        </v-col>
        <v-col sm="6">
          <h5>{{ $t(`consumptions.wheel3`) }} : {{ wheel3Info }}</h5>
          <h5>{{ $t(`consumptions.wheel4`) }} : {{ wheel4Info }}</h5>
        </v-col>
        <v-col sm="12" class="servoMotorAndTotal">
          <h5>{{ $t(`consumptions.total`) }} : {{ totalInfo }}</h5>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class BatteryCharge extends Vue {
  public robotConsumption!: RobotConsumption;

  public get wheel1Info() {
    return this.formatInfo(this.robotConsumption.wheel1);
  }
  public get wheel2Info() {
    return this.formatInfo(this.robotConsumption.wheel2);
  }
  public get wheel3Info() {
    return this.formatInfo(this.robotConsumption.wheel3);
  }
  public get wheel4Info() {
    return this.formatInfo(this.robotConsumption.wheel4);
  }
  public get totalInfo() {
    return this.formatInfo(this.robotConsumption.total);
  }
  public formatInfo(value: number) {
    if (value < 10) {
      return `${0 + value.toFixed(2)} W`;
    }
    return `${value.toFixed(2)} W`;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}

.servoMotorAndTotal {
  padding-top: 0.5em;
}
</style>
