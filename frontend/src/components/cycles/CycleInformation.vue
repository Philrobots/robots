<template>
  <v-card color="base">
    <v-card-title sm="12" class="step-title d-flex justify-center">
      <h3 class="white--text">{{ $t('cycles.cycleInformation') }}</h3>
    </v-card-title>
    <v-container>
      <v-row>
        <v-col sm="8" class="step-list">
          <StepList />
        </v-col>
        <v-spacer></v-spacer>
        <v-col sm="4">
          <v-container>
            <v-row>
              <v-col class="cycle-number-col">
                <CycleNumber />
              </v-col>
            </v-row>
            <v-row>
              <v-col>
                <Mode />
              </v-col>
            </v-row>
            <v-row>
              <v-col>
                <Chronometer />
              </v-col>
            </v-row>
          </v-container>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import CycleNumber from '@/components/cycles/CycleNumber.vue';
import StepList from '../cycles/StepList.vue';
import Mode from '../cycles/Mode.vue';
import Chronometer from '../cycles/Chronometer.vue';

@Component({
  components: {
    CycleNumber,
    StepList,
    Mode,
    Chronometer,
  },
})
export default class CycleInformation extends Vue {}
</script>

<style>
.step-title {
  padding: 4px !important;
  background: #1e1e1e;
}
.step-title > h3 {
  font-size: 1.1rem;
}
.step-list {
  padding-right: 1em !important;
}
.cycle-number-col {
  padding-top: 0 !important;
}
</style>
