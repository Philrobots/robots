<template>
  <v-card color="base">
    <v-card-title sm="12" class="consumption-title d-flex justify-center">
      <h3 class="white--text">
        {{ $t('consumptions.consumptionInformation') }}
      </h3>
    </v-card-title>
    <v-container>
      <v-row>
        <v-col sm="6">
          <BatteryCharge />
        </v-col>
        <v-col sm="6">
          <RobotConsumptionInfo />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import BatteryCharge from './BatteryCharge.vue';
import RobotConsumptionInfo from './RobotConsumptionInfo.vue';

@Component({
  components: {
    BatteryCharge: BatteryCharge,
    RobotConsumptionInfo: RobotConsumptionInfo,
  },
})
export default class ConsumptionInformation extends Vue {}
</script>

<style>
.consumption-title {
  padding: 4px !important;
  background: #1e1e1e;
}
.consumption-title > h3 {
  font-size: 1.1rem;
}
</style>
