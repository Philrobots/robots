<template>
  <v-card class="mainLayout">
    <v-row>
      <v-col sm="6">
        <!-- TODO : Layout should have components injected. The main view should add these components -->
        <cycle-information />
      </v-col>
      <v-col sm="6">
        <v-row>
          <v-col>
            <station-information />
          </v-col>
        </v-row>
        <v-row>
          <v-col>
            <consumption-information />
          </v-col>
        </v-row>
      </v-col>
      <v-col>
        <trajectories />
      </v-col>
    </v-row>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import ConsumptionInformation from '../components/consumptions/ConsumptionInformation.vue';
import CycleInformation from '../components/cycles/CycleInformation.vue';
import StationInformation from '../components/objectives/ObjectivesInformation.vue';
import Trajectories from '../components/trajectories/Trajectories.vue';

@Component({
  components: {
    ConsumptionInformation,
    CycleInformation,
    StationInformation,
    Trajectories,
  },
})
export default class MainLayout extends Vue {}
</script>

<style>
.mainLayout {
  background-color: rgba(0, 0, 0, 0) !important;
}
</style>
