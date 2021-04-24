import PuckDeposit from '@/components/objectives/PuckDeposit.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import { PuckList } from '@/types/puckList';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { PuckState } from '@/types/puckState';

const mockState = (puckStates: Array<PuckState>): State => {
  return {
    puckList: PuckListFactory.makeWithStates(puckStates),
  } as State;
};

describe('When mounting PuckDeposit component', () => {
  const wrapper = wrapComponentForTest(PuckDeposit);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given all pucks untouched yet', () => {
  const state = mockState(
    Array(PuckList.PUCKS_COUNT).fill(PuckState.UNTOUCHED)
  );

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should not have any puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(false);
    });
  });
});

describe('Given first puck gripped', () => {
  const state = mockState([
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should not have any puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(false);
      expect(pucks).toHaveLength(0);
    });
  });
});

describe('Given first puck released', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should have first puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(1);
    });
  });
});

describe('Given second puck gripped', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.GRIPPED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should have first puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(1);
    });
  });
});

describe('Given second puck released', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.UNTOUCHED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should have first and second puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(2);
    });
  });
});

describe('Given third puck gripped', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.GRIPPED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should have first and second puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(2);
    });
  });
});

describe('Given third puck released', () => {
  const state = mockState([
    PuckState.RELEASED,
    PuckState.RELEASED,
    PuckState.RELEASED,
  ]);

  describe('When mounting PuckDeposit', () => {
    const wrapper = wrapComponentForTest(PuckDeposit, state);

    it('Should have all puck released', () => {
      const pucks = wrapper.findAllComponents({ ref: 'puckDeposited' });

      expect(pucks.exists()).toBe(true);
      expect(pucks).toHaveLength(3);
    });
  });
});
