<template>
  <div>
    <v-btn
      :disabled="this.isDisabled"
      color="accent"
      @click="start"
      width="100%"
      elevation="6"
    >
      {{ $t('cycles.start') }}
    </v-btn>
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
export default class StartButton extends Vue {
  public cycleReady!: boolean;
  public cycleStarted!: boolean;

  public start() {
    this.$emit('start');
  }

  get isDisabled() {
    return !this.cycleReady || this.cycleStarted;
  }
}
</script>
