<template>
  <v-card color="base">
    <v-card-title sm="12" class="objectives-title d-flex justify-center">
      <h3 class="white--text">{{ $t('objectives.objectivesInformation') }}</h3>
    </v-card-title>
    <v-container>
      <v-row>
        <v-col sm="6">
          <Resistance />
        </v-col>
        <v-col sm="6">
          <FirstCorner />
        </v-col>
        <v-col sm="6">
          <GripState />
        </v-col>
        <v-col sm="6">
          <PuckDeposit />
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import Resistance from './/Resistance.vue';
import FirstCorner from './FirstCorner.vue';
import GripState from './GripState.vue';
import PuckDeposit from './/PuckDeposit.vue';

@Component({
  components: {
    Resistance: Resistance,
    FirstCorner: FirstCorner,
    GripState: GripState,
    PuckDeposit: PuckDeposit,
  },
})
export default class ObjectivesInformation extends Vue {}
</script>

<style>
/* TODO : All card titles should use the same CSS logic */
.objectives-title {
  padding: 4px !important;
  background: #1e1e1e;
}
.objectives-title > h3 {
  font-size: 1.1rem;
}
</style>
