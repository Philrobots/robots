<template>
  <div class="cycleNumber" ref="cycleNumber">
    <h3>{{ displayText }}</h3>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['cycleNumber']),
  },
})
export default class CycleNumber extends Vue {
  public cycleNumber!: number;

  get displayText() {
    return `${this.$t('cycles.cycle')} #${this.cycleNumber}`;
  }
}
</script>

<style>
.cycleNumber {
  text-align: center;
}
</style>
