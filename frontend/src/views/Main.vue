<template>
  <v-container>
    <div class="d-flex justify-center mt-5">
      <h1 class="font-weight-medium">{{ $t('appName') }}</h1>
    </div>
    <div class="d-flex justify-center mt-5">
      <main-layout />
    </div>
  </v-container>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';
import MainLayout from '../layouts/MainLayout.vue';
@Component({
  components: { MainLayout },
})
export default class Main extends Vue {}
</script>

<style></style>
