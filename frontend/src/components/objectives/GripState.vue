<template>
  <v-card class="lighten2">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('objectives.gripState') }}</h5>
    </v-card-title>
    <div
      ref="gripState"
      class="d-flex justify-center font-weight-bold"
      id="divChip"
    >
      <!-- TODO : Do not change color/opacity on hover -->
      <v-chip :color="puckInGrip ? 'green' : 'red'">
        <h3 class="white--text">
          {{
            puckInGrip ? $t('objectives.puckInGrip') : $t('objectives.noPuck')
          }}
        </h3></v-chip
      >
    </div>
  </v-card>
</template>
<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { PuckList } from '@/types/puckList';

@Component({
  computed: {
    ...mapState(['puckList']),
  },
})
export default class GripState extends Vue {
  public puckList!: PuckList;

  get puckInGrip() {
    return this.puckList.hasOneGripped;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0;
}

#divChip {
  padding: 0.3em;
}

#divChip .v-chip:hover::before {
  opacity: 0;
}
</style>
