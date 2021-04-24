import Vue from 'vue';
import VueI18n from 'vue-i18n';
import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";
import { messages, defaultLocale } from '@/i18n';
import { library } from '@fortawesome/fontawesome-svg-core';
import { fas } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome';
import { fab } from '@fortawesome/free-brands-svg-icons';

library.add(fas, fab);
Vue.component('font-awesome-icon', FontAwesomeIcon);

const customViewports = {
  station: {
    name: 'station',
    styles: {
      width: '1680px',
      height: '1050px',
    },
  },
};

export const parameters = {
  viewport: { viewports: customViewports },
  defaultViewport: customViewports.station,
}

Vue.use(VueI18n);
const i18n = new VueI18n({
   locale: defaultLocale,
   messages,
})

addDecorator(() => ({
  vuetify,
  i18n,
  template: '<v-app><story/></v-app>'
}))
