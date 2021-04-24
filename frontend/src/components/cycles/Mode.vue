<template>
  <div class="d-flex justify-center">
    <v-chip
      :color="this.color"
      class="white--text d-flex justify-center"
      ref="mode"
      id="mode-chip"
    >
      <h3>{{ $t(`cycles.modes.${this.actualMode}`) }}</h3>
    </v-chip>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
@Component({
  computed: {
    ...mapState(['cycleReady', 'cycleStarted']),
  },
})
export default class Mode extends Vue {
  public cycleReady!: boolean;
  public cycleStarted!: boolean;
  get actualMode() {
    if (!this.cycleReady) {
      return 'booting';
    } else if (this.cycleReady && this.cycleStarted) {
      return 'started';
    } else if (this.cycleReady) {
      return 'waiting';
    }
    return 'noInformation';
  }
  get color() {
    if (!this.cycleReady) {
      return 'red';
    }
    if (this.cycleReady && this.cycleStarted) {
      return 'green';
    } else if (this.cycleReady) {
      return 'amber';
    }
    return 'white';
  }
}
</script>

<style scoped>
#mode-chip {
  width: 100%;
}

#mode-chip:hover::before {
  opacity: 0;
}
</style>
