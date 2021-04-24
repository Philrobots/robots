<template>
  <v-card class="lighten2" height="100%">
    <v-card-title sm="6" class="lighten1 d-flex justify-center">
      <h5 class="white--text">{{ $t('objectives.firstCorner') }}</h5>
    </v-card-title>
    <v-container height="100%">
      <v-row align="center">
        <v-col sm="12">
          <div ref="corner" class="d-flex justify-center font-weight-bold">
            <v-badge
              dot
              v-if="this.isFirstCornerSet"
              :bottom="this.placementBottom"
              :left="this.placementLeft"
              :color="this.firstPuckColor"
            >
              {{ this.firstPuckCorner }}
            </v-badge>
          </div>
        </v-col>
      </v-row>
    </v-container>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';
import { Corner } from '@/types/corner';
import { PuckList } from '@/types/puckList';

@Component({
  computed: {
    ...mapState(['puckList']),
  },
})
export default class FirstCorner extends Vue {
  private puckList!: PuckList;

  private get placementLeft(): boolean {
    return (
      this.firstPuckCorner === Corner.A || this.firstPuckCorner === Corner.D
    );
  }

  private get placementBottom(): boolean {
    return (
      this.firstPuckCorner === Corner.C || this.firstPuckCorner === Corner.D
    );
  }

  private get firstPuckColor(): string {
    return this.puckList.first ? this.puckList.first.color : '';
  }

  private get firstPuckCorner(): string {
    return this.puckList.first.corner;
  }

  private get isFirstCornerSet(): boolean {
    return this.puckList.first && this.puckList.first.corner != Corner.UNSET;
  }
}
</script>

<style scoped>
.v-card__title {
  padding: 0;
}
</style>
