<template>
  <div>
    <v-btn
      :disabled="this.isDisabled"
      color="success"
      @click="reset"
      width="100%"
      elevation="6"
    >
      {{ $t('cycles.nextCycle') }}
    </v-btn>
  </div>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Step } from '@/types/step';

@Component({
  computed: {
    ...mapState(['currentStep']),
  },
})
export default class NextCycleButton extends Vue {
  public currentStep!: Step;

  public reset() {
    this.$emit('reset');
  }

  get isDisabled() {
    return this.currentStep !== Step.CycleEndedAndRedLedOn;
  }
}
</script>
