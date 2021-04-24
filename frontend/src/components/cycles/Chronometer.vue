<template>
  <div>
    <!-- TODO : v-row and v-col logic should be contained in CycleInformation -->
    <v-row>
      <v-col>
        <v-card class="lighten1 d-flex justify-center" ref="time">
          <h2>{{ this.updatedTime }}</h2>
        </v-card>
      </v-col>
    </v-row>
    <v-row>
      <v-col>
        <StartButton ref="button" @start="start" />
      </v-col>
    </v-row>
    <v-row>
      <v-col>
        <NextCycleButton ref="resetButton" @reset="reset" />
      </v-col>
    </v-row>
  </div>
</template>

<script lang="ts">
import { Step } from '@/types/step';
import { Component, Vue } from 'vue-property-decorator';
import { mapState, mapActions, mapMutations } from 'vuex';
import StartButton from './StartButton.vue';
import NextCycleButton from '@/components/cycles/NextCycleButton.vue';

@Component({
  components: {
    StartButton,
    NextCycleButton,
  },
  methods: {
    ...mapActions(['emitSocketStartCycle']),
    ...mapMutations(['RESET_CYCLE']),
  },
  computed: {
    ...mapState(['cycleReady', 'currentStep']),
  },
})
export default class Chronometer extends Vue {
  public emitSocketStartCycle!: () => void;
  public RESET_CYCLE!: () => void;
  public cycleReady!: boolean;
  public currentStep!: Step;

  public elapsedTime = 0;
  public interval: number | null = 0;
  public prevTime: number | null = 0;

  public start() {
    if (this.cycleReady || this.currentStep === Step.CycleEndedAndRedLedOn) {
      this.emitSocketStartCycle();

      if (!this.interval) {
        this.interval = setInterval(() => {
          if (this.currentStep === Step.CycleEndedAndRedLedOn) {
            this.stop();
          } else {
            if (!this.prevTime) {
              this.prevTime = Date.now();
            }

            this.elapsedTime += Date.now() - this.prevTime;
            this.prevTime = Date.now();

            this.updatedTime;
          }
        }, 50);
      }
    }
  }

  public stop() {
    if (this.interval) {
      clearInterval(this.interval);
      this.interval = null;
    }
    this.prevTime = null;
  }

  public reset() {
    this.RESET_CYCLE();
  }

  get updatedTime() {
    let tempTime = this.elapsedTime;
    const milliseconds = tempTime % 1000;
    tempTime = Math.floor(tempTime / 1000);
    const seconds = tempTime % 60;
    tempTime = Math.floor(tempTime / 60);
    const minutes = tempTime % 60;

    return `${minutes} : ${seconds}.${milliseconds}`;
  }
}
</script>

<style></style>
