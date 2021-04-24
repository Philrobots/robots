import Vue from 'vue';
import { EMIT_SOCKET_START_CYCLE } from './action-types';
import { START_CYCLE } from '@/store/mutation-types';

export const actions = {
  [EMIT_SOCKET_START_CYCLE]({ commit }: any) {
    Vue.prototype.$socket.client.emit('start_cycle');
    commit(START_CYCLE);
  },
};
