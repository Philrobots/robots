<template>
  <v-card class="lighten2">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('objectives.resistance') }}</h5>
    </v-card-title>
    <v-container>
      <v-row align="center">
        <v-col sm="5">
          <div
            ref="resistanceValue"
            class="d-flex justify-center font-weight-bold"
          >
            {{ this.resistance }} Ω
          </div>
        </v-col>
        <v-col sm="7">
          <v-avatar
            ref="pucks"
            size="30"
            v-for="(puck, i) in this.pucksToDisplay"
            :key="i"
            :color="puck.color.toString()"
            class="lighten3--text font-weight-bold"
          >
            {{ puck.number }}
          </v-avatar>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { PuckList } from '@/types/puckList';
import { Color } from '@/types/color';

@Component({
  computed: {
    ...mapState(['resistance', 'puckList']),
  },
})
export default class Resistance extends Vue {
  public resistance!: number;
  public puckList!: PuckList;

  get pucksToDisplay() {
    return this.puckList.pucks.filter((puck) => puck.color !== Color.Unset);
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0em;
}
</style>
