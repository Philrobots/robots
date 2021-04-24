<template>
  <h2 class="primary--text" ref="time">
    {{ formatedTime }}
  </h2>
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
export default class RemainingTime extends Vue {
  public robotConsumption!: RobotConsumption;

  private get timeInSeconds() {
    return this.robotConsumption.remainingTime;
  }

  private get formatedTime() {
    return new Date(this.timeInSeconds * 1000).toISOString().substr(11, 8);
  }
}
</script>
