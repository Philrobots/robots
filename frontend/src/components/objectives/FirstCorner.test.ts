import FirstCorner from '@/components/objectives/FirstCorner.vue';
import wrapComponentForTest from '../../util/wrapComponentForTest';
import { State } from '@/store/state';
import { PuckListFactory } from '@/factories/PuckListFactory';
import { Corner } from '@/types/corner';

describe('When mounting FirstCorner component', () => {
  const wrapper = wrapComponentForTest(FirstCorner);

  it('Should mount', () => {
    expect(wrapper.vm).toBeTruthy();
  });
});

describe('Given state', () => {
  const state = {
    puckList: PuckListFactory.make(),
  } as State;

  describe('When mounting FirstCorner', () => {
    const wrapper = wrapComponentForTest(FirstCorner, state);

    it('Should contains the right letter of corner', () => {
      const expectedCorner =
        state.puckList.first.corner === Corner.UNSET
          ? ''
          : state.puckList.first.corner;

      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe(expectedCorner);
    });
  });
});

describe('Given no state', () => {
  const wrapper = wrapComponentForTest(FirstCorner);

  describe('When mounting FirstCorner component without props', () => {
    it('Should not contains resistanceValue', () => {
      const letterCorner = wrapper.findComponent({ ref: 'corner' });

      expect(letterCorner.exists()).toBe(true);
      expect(letterCorner.text()).toBe('');
    });
  });
});
