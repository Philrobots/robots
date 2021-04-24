import Vue from 'vue';
import VueRouter from 'vue-router';
import VueI18n from 'vue-i18n';
import App from './App.vue';
import vuetify from '@/plugins/vuetify';
import '@/plugins/font-awesome';
import store from './store';
import { io } from 'socket.io-client';
import VueSocketIOExt from 'vue-socket.io-extended';
import { messages, defaultLocale } from './i18n';

Vue.config.productionTip = false;

const socket = io(process.env.VUE_APP_STATION_URL);
Vue.use(VueSocketIOExt, socket, { store });

Vue.use(VueRouter);
Vue.use(VueI18n);

const locale = window.location.pathname.replace(/^\/([^/]+).*/i, '$1');

const i18n = new VueI18n({
  locale: locale.trim().length && locale != '/' ? locale : defaultLocale,
  fallbackLocale: 'en',
  messages,
});

const routes = [
  {
    path: '/',
    component: require('./views/Main'),
    meta: {
      title: i18n.t('appName'),
    },
  },
];

const router = new VueRouter({
  base: locale.trim().length && locale != '/' ? '/' + locale : undefined,
  mode: 'history',
  routes,
});

router.beforeEach((to, from, next) => {
  document.title = to.meta.title;
  next();
});

new Vue({
  vuetify,
  store,
  router,
  i18n,
  render: (h) => h(App),
}).$mount('#app');
